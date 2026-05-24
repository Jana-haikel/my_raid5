/* Most of the following headerfiles are defined within the kernel space not the userspace, 
   therefore they will be inaccessible unless compiled the proper way with a Makefile 
   pointing towards the kernel-devel-tree tools to create the LKM (loadable kernel module)
   WITHIN the kernel space.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <linux/slab.h>
#include <linux/device-mapper.h>
#include <linux/highmem.h>
#include <linux/mutex.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/gfp.h>  
#include <linux/completion.h>
#include <linux/atomic.h> 
#include <linux/ktime.h> 
#include <asm/fpu/api.h>


#define CHUNK_SIZE      (64 * 1024) //data package will be handled into 64KB chunks
#define CHUNK_SECTORS   (CHUNK_SIZE / 512)      // 128 sectors per data chunk
#define NUM_DISKS       3 //total number of brd devices 
#define DATA_DISKS      2 //third one will hold parity calculation
#define TILE_SIZE (16 * 1024)   // 16KB — optimal for P-core L1D (16*3)
#define STRIPE_DATA_SECTORS (CHUNK_SECTORS * DATA_DISKS)  // 256 sectors = 128KB

struct raid5_map { //temporary holding place for where everything will be written for each I/O request
    int      data_disk[DATA_DISKS];
    int      parity_disk;
    sector_t disk_sector;
};

struct raid5_device { 
    struct gendisk *disk; //generic disk abstraction (to submit this as a driver in the vfs)
    struct block_device *bdev[NUM_DISKS]; //track an array of all 3 brd devices (in an array of pointers)
    struct file *bdev_file[NUM_DISKS]; //vfs file handle system to handle data files sent to block layer
    /*VFS handle object has {mode (permissions), reference lock (device is freed when this hits 0), 
      private data pointer (of *bdev)} */
    struct workqueue_struct *wq; //a pointer to a kernel background workqueue (would function as a thread pool)
    
    // internal performance counters
    atomic64_t parity_calls;
    atomic64_t parity_bytes;
    atomic64_t parity_ns;
    atomic64_t write_calls;
    atomic64_t write_ns;
};

struct raid5_work { //package the request bio to be sent to the workqueue and handled simioultaneously 
    struct work_struct work;
    struct bio *bio;
};

static struct raid5_device *g_dev; //global pointer to handle the raid5 device
static int my_major; //major number (for registration in /proc/devices/)


//mapping and striping logic (1 stripe spans across all 3 disks)
static void raid5_map_sector(sector_t sector, struct raid5_map *map) {
    sector_t stripe = sector / (CHUNK_SECTORS * DATA_DISKS); //stripe no. = sector no. / 256 
    sector_t offset = sector % (CHUNK_SECTORS * DATA_DISKS); //offset inside the stripe's sector

    int slot = offset / CHUNK_SECTORS; //is it in first/second data "slot"
    sector_t chunk_off = offset % CHUNK_SECTORS; //offset inside the chunk

    map->parity_disk = NUM_DISKS - 1 - (stripe % NUM_DISKS); //handles rotating parity
    map->disk_sector = (stripe * CHUNK_SECTORS) + chunk_off; //offsets the global sector address locally

    for (int i = 0; i < DATA_DISKS; i++) //fill data disks (skip parity disk)
        map->data_disk[i] = i >= map->parity_disk ? i + 1 : i; //this is essential since parity rotates over 3 disks
}

static void xor_avx2_inner(u8 *dst, const u8 *src1, const u8 *src2, size_t len) {
    size_t i = 0;

    //vector register holds 32 bytes of data
    //unrolling by a factor of 8: (8*32) = 256 bytes per iteration
    // hides L1 cache load latency fully on P-cores
    for (; i + 256 <= len; i += 256) {
        //packaged as assembly code
        asm volatile(
            // load 256 bytes from src1
            "vmovdqu   0(%1), %%ymm0\n\t"
            "vmovdqu  32(%1), %%ymm1\n\t"
            "vmovdqu  64(%1), %%ymm2\n\t"
            "vmovdqu  96(%1), %%ymm3\n\t"
            "vmovdqu 128(%1), %%ymm4\n\t"
            "vmovdqu 160(%1), %%ymm5\n\t"
            "vmovdqu 192(%1), %%ymm6\n\t"
            "vmovdqu 224(%1), %%ymm7\n\t"
            // XOR with 256 bytes from src2
            "vpxor   0(%2), %%ymm0, %%ymm0\n\t"
            "vpxor  32(%2), %%ymm1, %%ymm1\n\t"
            "vpxor  64(%2), %%ymm2, %%ymm2\n\t"
            "vpxor  96(%2), %%ymm3, %%ymm3\n\t"
            "vpxor 128(%2), %%ymm4, %%ymm4\n\t"
            "vpxor 160(%2), %%ymm5, %%ymm5\n\t"
            "vpxor 192(%2), %%ymm6, %%ymm6\n\t"
            "vpxor 224(%2), %%ymm7, %%ymm7\n\t"
            // store 256 bytes to dst
            "vmovdqu %%ymm0,   0(%0)\n\t"
            "vmovdqu %%ymm1,  32(%0)\n\t"
            "vmovdqu %%ymm2,  64(%0)\n\t"
            "vmovdqu %%ymm3,  96(%0)\n\t"
            "vmovdqu %%ymm4, 128(%0)\n\t"
            "vmovdqu %%ymm5, 160(%0)\n\t"
            "vmovdqu %%ymm6, 192(%0)\n\t"
            "vmovdqu %%ymm7, 224(%0)\n\t"
            :
            : "r"(dst + i), "r"(src1 + i), "r"(src2 + i)
            : "ymm0","ymm1","ymm2","ymm3",
              "ymm4","ymm5","ymm6","ymm7","memory"
        );
    }

    //if the remainder was just a bit smaller than 256
    // 4-way fallback for remaining 128-byte chunks 
    for (; i + 128 <= len; i += 128) {
        asm volatile(
            "vmovdqu   0(%1), %%ymm0\n\t"
            "vmovdqu  32(%1), %%ymm1\n\t"
            "vmovdqu  64(%1), %%ymm2\n\t"
            "vmovdqu  96(%1), %%ymm3\n\t"
            "vpxor   0(%2), %%ymm0, %%ymm0\n\t"
            "vpxor  32(%2), %%ymm1, %%ymm1\n\t"
            "vpxor  64(%2), %%ymm2, %%ymm2\n\t"
            "vpxor  96(%2), %%ymm3, %%ymm3\n\t"
            "vmovdqu %%ymm0,   0(%0)\n\t"
            "vmovdqu %%ymm1,  32(%0)\n\t"
            "vmovdqu %%ymm2,  64(%0)\n\t"
            "vmovdqu %%ymm3,  96(%0)\n\t"
            :
            : "r"(dst + i), "r"(src1 + i), "r"(src2 + i)
            : "ymm0","ymm1","ymm2","ymm3","memory"
        );
    }

    // scalar processing for any remaining bytes
    for (; i < len; i++)
        dst[i] = src1[i] ^ src2[i];
}

/* cache-blocked wrapper — tiles the AVX2 loop to keep
   data hot in L1 cache, then processes timing */
static void compute_parity(u8 *parity, const u8 *d0,
                            const u8 *d1, size_t len)
{
    size_t offset = 0;
    ktime_t start = ktime_get();

    kernel_fpu_begin();
    while (offset < len) { //len is 64kb
        size_t tile = min_t(size_t, TILE_SIZE, len - offset); //set tile window 16kb
        xor_avx2_inner(parity + offset, d0 + offset, d1 + offset, tile); //process each tile window
        offset += tile;
    }
    kernel_fpu_end();

    atomic64_add(ktime_to_ns(ktime_sub(ktime_get(), start)),
                 &g_dev->parity_ns);
    atomic64_inc(&g_dev->parity_calls);
    atomic64_add(len, &g_dev->parity_bytes);
}

//allocate a physically contiguous 64KB buffer
static u8 *alloc_chunk_buf(void) {
    return (u8 *)__get_free_pages(GFP_NOIO | __GFP_ZERO, get_order(CHUNK_SIZE));
}

static void free_chunk_buf(u8 *buf) {
    if (buf)
        free_pages((unsigned long)buf, get_order(CHUNK_SIZE));
}

// read/write between an underlying bdev and a flat kernel buffer
static int bdev_rw_buf(struct block_device *bdev, sector_t sector, u8 *buf, unsigned int op)
{
    struct bio *bio;
    int i, err;
    int nr_pages = CHUNK_SIZE / PAGE_SIZE;  // 16 pages for 64KB

    bio = bio_alloc(bdev, nr_pages, op, GFP_NOIO);
    bio->bi_iter.bi_sector = sector;

    for (i = 0; i < nr_pages; i++)
        bio_add_page(bio, virt_to_page(buf + i * PAGE_SIZE), PAGE_SIZE, 0);

    err = submit_bio_wait(bio);
    bio_put(bio);
    return err;
}

// copy bytes out of a bio into a flat kernel buffer
// skip = how many bytes to skip from the start of the bio
static void bio_copy_to_buf(struct bio *bio, u8 *buf, size_t skip, size_t len)
{
    struct bio_vec bv;
    struct bvec_iter iter;
    size_t copied = 0;

    bio_for_each_segment(bv, bio, iter) {
        void *kaddr;
        size_t to_copy;

        if (skip >= bv.bv_len) {
            skip -= bv.bv_len;
            continue;
        }

        to_copy = min_t(size_t, bv.bv_len - skip, len - copied);
        kaddr = kmap_local_page(bv.bv_page);
        memcpy(buf + copied, kaddr + bv.bv_offset + skip, to_copy);
        kunmap_local(kaddr);

        copied += to_copy;
        skip = 0;
        if (copied >= len)
            break;
    }
}

//context for multiple executing threads at once
struct raid5_io_ctx {
    atomic_t          remaining;   // counts pending bios
    struct completion done;        // fires when remaining hits 0
    blk_status_t      status;      // captures any error
};

static void raid5_end_io(struct bio *bio) {
    struct raid5_io_ctx *ctx = bio->bi_private;

    if (bio->bi_status)
        ctx->status = bio->bi_status;

    // last bio to complete wakes the waiter
    if (atomic_dec_and_test(&ctx->remaining))
        complete(&ctx->done);

    bio_put(bio);
}

static struct bio *build_write_bio(struct block_device *bdev, sector_t sector, u8 *buf, struct raid5_io_ctx *ctx) {
    struct bio *bio;
    int i;
    int nr_pages = CHUNK_SIZE / PAGE_SIZE;

    bio = bio_alloc(bdev, nr_pages, REQ_OP_WRITE, GFP_NOIO);
    bio->bi_iter.bi_sector = sector;
    bio->bi_end_io = raid5_end_io;
    bio->bi_private = ctx;

    for (i = 0; i < nr_pages; i++)
        bio_add_page(bio, virt_to_page(buf + i * PAGE_SIZE),PAGE_SIZE, 0);

    return bio;
}

// submit all 3 disk writes simultaneously and wait once
static int parallel_write_stripe(struct raid5_map *map, u8 *d0, u8 *d1, u8 *parity) {
    struct raid5_io_ctx ctx;
    struct bio *bios[3];

    atomic_set(&ctx.remaining, 3);
    init_completion(&ctx.done);
    ctx.status = BLK_STS_OK;

    bios[0] = build_write_bio(g_dev->bdev[map->data_disk[0]], map->disk_sector, d0, &ctx);
    bios[1] = build_write_bio(g_dev->bdev[map->data_disk[1]], map->disk_sector, d1, &ctx);
    bios[2] = build_write_bio(g_dev->bdev[map->parity_disk], map->disk_sector, parity, &ctx);

    // all 3 in flight simultaneously
    submit_bio(bios[0]);
    submit_bio(bios[1]);
    submit_bio(bios[2]);

    wait_for_completion(&ctx.done);

    return ctx.status == BLK_STS_OK ? 0 : -EIO;
}

static bool is_full_stripe_write(struct bio *bio, sector_t stripe)
{
    sector_t stripe_start = stripe * STRIPE_DATA_SECTORS;

    // must start at stripe boundary and cover full stripe
    return bio->bi_iter.bi_sector == stripe_start &&
           bio->bi_iter.bi_size  >= (CHUNK_SIZE * DATA_DISKS);
}

// FAST PATH: no reads needed — extract parity directly from incoming data
static int raid5_full_stripe_write(struct bio *bio, struct raid5_map *map)
{
    u8 *d0 = NULL, *d1 = NULL, *parity = NULL;
    int err = -ENOMEM;

    d0     = alloc_chunk_buf();
    d1     = alloc_chunk_buf();
    parity = alloc_chunk_buf();

    if (!d0 || !d1 || !parity)
        goto out;

    // pull both data chunks straight out of the bio
    bio_copy_to_buf(bio, d0, 0,          CHUNK_SIZE);
    bio_copy_to_buf(bio, d1, CHUNK_SIZE, CHUNK_SIZE);

    // compute parity — AVX2 + unrolled + cache-blocked
    compute_parity(parity, d0, d1, CHUNK_SIZE);

    // write D0, D1, P all in parallel
    err = parallel_write_stripe(map, d0, d1, parity);

out:
    free_chunk_buf(d0);
    free_chunk_buf(d1);
    free_chunk_buf(parity);
    return err;
}

// SLOW PATH: partial write — must read existing data first
static int raid5_rmw_write(struct bio *bio, struct raid5_map *map)
{
    u8 *d0 = NULL, *d1 = NULL, *parity = NULL;
    int err = -ENOMEM;

    d0     = alloc_chunk_buf();
    d1     = alloc_chunk_buf();
    parity = alloc_chunk_buf();

    if (!d0 || !d1 || !parity)
        goto out;

    // read existing full chunks from both data disks
    err = bdev_rw_buf(g_dev->bdev[map->data_disk[0]], map->disk_sector, d0, REQ_OP_READ);
    if (err) goto out;

    err = bdev_rw_buf(g_dev->bdev[map->data_disk[1]], map->disk_sector, d1, REQ_OP_READ);
    if (err) goto out;

    // overlay the partial new data from the bio onto d0
    bio_copy_to_buf(bio, d0, 0, min_t(size_t, bio->bi_iter.bi_size, CHUNK_SIZE));

    // recompute parity from complete updated chunks
    compute_parity(parity, d0, d1, CHUNK_SIZE);

    err = parallel_write_stripe(map, d0, d1, parity);

out:
    free_chunk_buf(d0);
    free_chunk_buf(d1);
    free_chunk_buf(parity);
    return err;
}

static void raid5_work_fn(struct work_struct *work)
{
    struct raid5_work *rw = container_of(work, struct raid5_work, work);
    struct bio *bio = rw->bio;
    struct raid5_map map;
    sector_t stripe;
    int err;
    ktime_t start = ktime_get(); 
    kfree(rw);


    raid5_map_sector(bio->bi_iter.bi_sector, &map);
    stripe = bio->bi_iter.bi_sector / STRIPE_DATA_SECTORS;

    if (is_full_stripe_write(bio, stripe))
        err = raid5_full_stripe_write(bio, &map);
    else
        err = raid5_rmw_write(bio, &map);

    atomic64_add(ktime_to_ns(ktime_sub(ktime_get(), start)), &g_dev->write_ns);
    atomic64_inc(&g_dev->write_calls);

    if (err)
        bio_io_error(bio);
    else
        bio_endio(bio);
}

//intercepts the native submit_bio(..) function and redirects it here
static void raid5_submit_bio(struct bio *bio) {
    struct raid5_map map;

    if (unlikely(op_is_discard(bio->bi_opf))) {
        bio_endio(bio);
        return;
    }

    raid5_map_sector(bio->bi_iter.bi_sector, &map);

    if (op_is_write(bio->bi_opf)) {
        struct raid5_work *rw = kmalloc(sizeof(*rw), GFP_NOIO);
        if (!rw) {
            bio_io_error(bio);
            return;
        }
        INIT_WORK(&rw->work, raid5_work_fn);
        rw->bio = bio;
        queue_work(g_dev->wq, &rw->work);
        return;
    }

    // reads go straight to the correct data disk
    bio_set_dev(bio, g_dev->bdev[map.data_disk[0]]);
    bio->bi_iter.bi_sector = map.disk_sector;
    submit_bio(bio);

}

static void raid5_print_stats(void)
{
    u64 pcalls = atomic64_read(&g_dev->parity_calls);
    u64 pbytes = atomic64_read(&g_dev->parity_bytes);
    u64 pns    = atomic64_read(&g_dev->parity_ns);
    u64 wcalls = atomic64_read(&g_dev->write_calls);
    u64 wns    = atomic64_read(&g_dev->write_ns);

    if (pns > 0)
        pr_info("ramraid parity: calls=%llu bytes=%llu time=%llu ns  ~%llu GB/s\n",
                pcalls, pbytes, pns, (pbytes * 1000) / pns);

    if (wcalls > 0)
        pr_info("ramraid writes: calls=%llu avg_latency=%llu ns\n",
                wcalls, wns / wcalls);
}


//whenever there is a request for this device, send it to my raid5_submit_bio(..) function
static const struct block_device_operations raid5_fops = {
    .owner      = THIS_MODULE,
    .submit_bio = raid5_submit_bio,
};

static int __init raid5_init(void) {
    int i, err;
    struct gendisk *disk; //temporary pointer to the drive (before it's saved in the dev struct)
    struct queue_limits lim = { //hardware rules
        .physical_block_size = CHUNK_SIZE, //64KB
        .features = BLK_FEAT_SYNCHRONOUS, //synchronous I/O operations
    };

    g_dev = kzalloc(sizeof(struct raid5_device), GFP_KERNEL); //allocate kernel memory for the device (pre-filled with 0s)
    if (!g_dev)
        return -ENOMEM;

    const char *paths[NUM_DISKS] = { //brd device paths
    "/dev/rambuff0",
    "/dev/rambuff1",
    "/dev/rambuff2"
    };

    for (i = 0; i < NUM_DISKS; i++) {
        //opening the brd devices (anchoring them to handle requests directly when needed)
        g_dev->bdev_file[i] = bdev_file_open_by_path(paths[i], BLK_OPEN_READ | BLK_OPEN_WRITE, NULL, NULL); 
        //requests r/w access permissions from vfs (rw flags)

        if (IS_ERR(g_dev->bdev_file[i])) { //if the vfs handle operation failed
            err = PTR_ERR(g_dev->bdev_file[i]);
            goto out_close;
        }

        //extract the hardware structure pointer from the vfs file struct, and save it into the raid5_device struct:
        g_dev->bdev[i] = file_bdev(g_dev->bdev_file[i]);
    }

    //registering major number
    my_major = register_blkdev(0, "my_raid5");
    if (unlikely(my_major < 0)) {
        err = my_major;
        goto out_close;
    } 

    //allocate memory for gendisk "Create gendisk object" (i.e /dev/my_raid5)
    disk = blk_alloc_disk(&lim, NUMA_NO_NODE); //pass the hw rules,and follow NUMA (allocate the memory anywhere)
    if (IS_ERR(disk)) {                       
        err = PTR_ERR(disk);
        goto out_unregister;
    }                       
    //virtual drive's details:
    g_dev->disk = disk;
    disk->major = my_major; //link registered major number
    disk->first_minor = 0; //start counting from 0
    disk->minors = 1; //cannot be internally partitioned
    disk->fops = &raid5_fops; //set flags
    disk->private_data = g_dev; //set private data pointer
    snprintf(disk->disk_name, DISK_NAME_LEN, "my_raid5");

    // capacity = 2/3 of total (one disk is always parity)
    // each rambuff is rd_size*2 sectors
    set_capacity(disk, CHUNK_SECTORS * DATA_DISKS * (get_capacity(g_dev->bdev[0]->bd_disk) / CHUNK_SECTORS));

    g_dev->wq = alloc_workqueue("raid5_wq", WQ_MEM_RECLAIM, 0);
    if (!g_dev->wq) {
        err = -ENOMEM;
        goto out_put_disk;
    }
    
    err = add_disk(disk);
    if (err)
        goto out_put_disk;

    pr_info("raid5: loaded, /dev/my_raid5 ready\n");
    return 0;


    out_del_wq:
    destroy_workqueue(g_dev->wq);

    out_put_disk:
    put_disk(disk); //frees gendisk memory if add_disk failed

    out_unregister:
        unregister_blkdev(my_major, "my_raid5"); //frees major device number

    out_close:
        for (i = 0; i < NUM_DISKS; i++)
            if (g_dev->bdev_file[i] && !IS_ERR(g_dev->bdev_file[i]))
                bdev_fput(g_dev->bdev_file[i]); // frees opened brd disks

    kfree(g_dev); //frees the global state tracker structure
    return err; //passes the error code

}

static void __exit raid5_exit(void) {
    raid5_print_stats();

    destroy_workqueue(g_dev->wq);
    del_gendisk(g_dev->disk); //stops the virtual disk driver from running
    put_disk(g_dev->disk); //free its memory (decrease its refcount to 0)
    unregister_blkdev(my_major, "my_raid5"); //unregister major number
    for (int i = 0; i < NUM_DISKS; i++)
        bdev_fput(g_dev->bdev_file[i]); //free each of the brd devices
    kfree(g_dev); //free the struct of the device from kernel memory (allocated via kzalloc(..))
    pr_info("raid5: Module unloaded with no issues <3 \n");
}

module_init(raid5_init);
module_exit(raid5_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RAID5 over Ram disks Project.");