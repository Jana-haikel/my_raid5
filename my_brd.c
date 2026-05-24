/* Most of the following headerfiles are defined within the kernel space not the userspace, 
   therefore they will be inaccessible unless compiled the proper way with a Makefile 
   pointing towards the kernel-devel-tree tools to create the LKM (loadable kernel module)
   WITHIN the kernel space.
 */

#include <linux/init.h>
#include <linux/initrd.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/major.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <linux/highmem.h>
#include <linux/mutex.h>
#include <linux/pagemap.h>
#include <linux/xarray.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/backing-dev.h>
#include <linux/debugfs.h>

#include <linux/uaccess.h>


static int my_major; //let the kernel dynamically assign a major number
unsigned long rd_size = 2097152; //each brd size 2GB; with dynamic allocation
struct my_brd_device *my_devices;

struct my_brd_device {
    int brd_number; //minor
    struct gendisk *brd_disk;
    struct xarray page_tree;
    u64 tot_nr_pages;
};


//read a page from storage "originally named brd_lookup_page(...) in the kernel source code"
static struct page *page_search(struct my_brd_device *brd, sector_t sector) {    
    struct page *page; //ptr to page we're attempting to search for
    XA_STATE(xas, &brd->page_tree, sector >> PAGE_SECTORS_SHIFT); //cursor to search for page at certain index

    rcu_read_lock(); // Read-Copy-Update synchronization pattern for concurrent reads, the lock ensures data validity
    repeat:
        page = xas_load(&xas); //attempt lookup
        // xas_retry handles xarray in a transient state; aka it's being modified by another thread using xa_lock
        if(xas_retry(&xas, page)) {
            //xa_reset goes back to the top of the xarray and attempts search again
            xas_reset(&xas);
            goto repeat;
        }

        if (!page)
            goto out;
        
        if(!get_page_unless_zero(page)) {
            //if page reference count hits 0, the page gets freed, in that case, reset xas again.
            xas_reset(&xas);
            goto repeat;
        }

        //unlikely is a compiler performance hint for branch prediction that indicates this scenario is not likely to occur.
        if (unlikely(page != xas_reload(&xas))) {
            /* make sure the page we got is the same page requested 
            (prevents the race condition where a page is modified exactly before it is fetched by the cpu) */

            put_page(page); //free the wrong page
            xas_reset(&xas); //reset the xarray state to the top again
            goto repeat; //repeat and re-attempt search
        }

    out:
        rcu_read_unlock();
        return page;

}


//allocate a new page for a sector in the brd
static struct page *insert_new_page(struct my_brd_device *brd, sector_t sector, blk_opf_t opf) {

    /* if the I/O request is non-blocking "DON'T WAIT", and there was no RAM space, just fail and return,
       DO NOT start another I/O operation (prevents a Recursive Deadlock)  
    */
    gfp_t gfp = (opf &REQ_NOWAIT) ? GFP_NOWAIT : GFP_NOIO;
    struct page *page, *ret;
    page = alloc_page (gfp | __GFP_ZERO | __GFP_HIGHMEM); /* allocate a free page with all zeroes, and it can be anywhere even high memory 
                                                             (sometimes the kernel cannot see high memory) */
    if (!page)
        return ERR_PTR(-ENOMEM);

    xa_lock(&brd->page_tree); //lock the xarray as it is being modified (adding a page) "Transient state"

    //compare and exchange, only insert page if the slot is NULL, this handles the (Check then Act race condition)
    ret = __xa_cmpxchg(&brd->page_tree, sector >> PAGE_SECTORS_SHIFT, NULL, page, gfp); 

    if (!ret) { //if slot is indeed empty (NULL)
        brd->tot_nr_pages++; //increment total number of pages
        get_page(page); //increment page reference counter
        xa_unlock(&brd->page_tree); //unlock the xarray after modification
        return page;
    }

    if (!xa_is_err(ret)) { //if slot contains pointer to an existing page
        get_page(ret); //increase ret's ref counter
        xa_unlock(&brd->page_tree);
        put_page(page); //free the page we originally allocated (since another one exists in that slot)
        return ret; //return that page's ptr
    }

    xa_unlock(&brd->page_tree);
    put_page(page);
    return ERR_PTR(xa_err(ret)); //system error handling

}

//free up all pages and delete the xarray
static void brd_free_pages(struct my_brd_device *brd) {
    struct page *page;
    pgoff_t idx;

    xa_for_each(&brd->page_tree, idx, page) {
        put_page(page);
        cond_resched(); //for big loops, don't stall the system, reschedule accordingly
    }
    xa_destroy(&brd->page_tree);
}


// The main read/write traffic handler
static bool rw_biovec(struct my_brd_device *brd, struct bio *bio) {

    struct bio_vec bv = bio_iter_iovec(bio, bio->bi_iter); /* the chunk of data the user needs to read/write.
                                                              This data is saved in a vector inside the bio struct */
    sector_t sector = bio->bi_iter.bi_sector;
    u32 offset = (sector & (PAGE_SECTORS-1)) << SECTOR_SHIFT; // page offset
    blk_opf_t opf = bio->bi_opf; //bio operation flag
    struct page *page;
    void *kaddr; //hold a pointer to kernel address space to copy from user to kernel

    bv.bv_len = min_t(u32, bv.bv_len, PAGE_SIZE - offset); //ensures you never read/write data bigger than a page

    page = page_search(brd, sector); //always attempt to search for a page first, then handle r/w

    if (!page && op_is_write(opf)) { //if page not found and flag is write
        page = insert_new_page(brd, sector, opf);
        if (IS_ERR(page))
            goto out_error;
    }

    kaddr = bvec_kmap_local(&bv); //map the user's data buffer to a temporary kernel virtual address space
    
    if (op_is_write(opf)) { //if flag set to write
        memcpy_to_page(page, offset, kaddr, bv.bv_len); //copy from user's buffer to Ram disk
    } else {
        if (page) //it's a read request
            memcpy_from_page(kaddr, page, offset, bv.bv_len); //copy from Ram disk data to user's buffer
        else
            memset(kaddr, 0, bv.bv_len); // reading from empty/NULL page returns all 0
    
    }

    kunmap_local(kaddr); //unmap the kernel virtual address space
    bio_advance_iter_single(bio, &bio->bi_iter, bv.bv_len); //advance iterator to next chunk

    if (page)
        put_page(page); //decrement the ref counter so the page can be freed when it hits 0

    return true; //success


out_error:
    if (PTR_ERR(page) == -ENOMEM && (opf & REQ_NOWAIT))
        bio_wouldblock_error(bio); //the flag says "don't wait" and this'd cause stall, terminate and throw this error
    else
        bio_io_error(bio);

    return false; //failure

}

static void brd_submit_bio(struct bio *bio) {
    struct my_brd_device *brd = bio->bi_bdev->bd_disk->private_data; //lets the driver know which device it's working on
                                //which block in which disk and where the data is (all for the bio)
    if (unlikely(op_is_discard(bio->bi_opf))) {
        //discard logic to be added later
        bio_endio(bio);
        return;
    }

    do {
        if (!rw_biovec(brd, bio))
            return;
    } while (bio->bi_iter.bi_size); //iterate on the entire chunk the user needs to read/write 

    bio_endio(bio);
}

//whenever there is a request for this device, send it to my submit_bio function:
static const struct block_device_operations brd_fops = {
 
    .owner =                 THIS_MODULE,
    .submit_bio =            brd_submit_bio,
};

static int brd_alloc(struct my_brd_device *brd, int index) {

    struct gendisk *disk; //generic disk; kernel's hw abstraction unit

    struct queue_limits lim = { //physical (hardware) rules
        .physical_block_size = PAGE_SIZE, //align size to 4KB pages
        .features = BLK_FEAT_SYNCHRONOUS | BLK_FEAT_NOWAIT, //I/O is synch. REQ_NOWAIT, used for optimized I/O (faster requests)
    };

    brd->brd_number = index;
    xa_init(&brd->page_tree);
    brd->tot_nr_pages = 0;

    disk = blk_alloc_disk(&lim, NUMA_NO_NODE); //create gendisk object; public interface; i.e /dev/brd0
    if (IS_ERR(disk))
        return PTR_ERR(disk);

    brd->brd_disk = disk;
    disk->major = my_major;
    disk->first_minor = index; // ex. when index is 0, get disk 0
    disk->minors = 1; //each disk cannot be partitioned; only 1 minor each
    disk->fops = &brd_fops; //flag set
    disk->private_data = brd; //holds data

    snprintf(disk->disk_name, DISK_NAME_LEN, "rambuff%d", index); //how the disk will appear in the /dev/ folder
    set_capacity(disk, rd_size*2); //rd_size is in KB; 2 sectors; 512*2 for a full KB. capacity is set in sectors

    return add_disk(disk);
}

static void cleanup(void) {

    for (int i = 0; i < 3; ++i) {
        struct my_brd_device *brd = &my_devices[i];
        if (brd->brd_disk) {
            del_gendisk(brd->brd_disk); //safety check
            put_disk(brd->brd_disk);
        }
        brd_free_pages(brd); //free the xarray and all pages in the brd
    }
    kfree(my_devices); //free the array with the 3 devices itself, allocated in the init function
}

static int __init brd_init(void) {

    int i, err;

    //creating an array for 3 disks
    my_devices = kzalloc(sizeof(struct my_brd_device) * 3, GFP_KERNEL);
    if (!my_devices)
        return -ENOMEM;

    //registering major number
    my_major = register_blkdev(0, "RAID5_RAM_BUFF");
    if (unlikely(my_major < 0)) {
        kfree(my_devices);
        return -EIO;  // no free major numbers (an extremely rare case)
    }

    //static allocation for an array of 3 brds
    for (i = 0; i < 3; ++i) {
        err = brd_alloc(&my_devices[i], i);
        if (err) {
            goto out_cleanup; //failed allocation
        }
    }

    pr_info("Module loaded successfully.\n");
    return 0;

out_cleanup:
    cleanup();
    unregister_blkdev(my_major, "RAID5_RAM_BUFF");
    return err;
}


static void __exit brd_exit(void) {

    unregister_blkdev(my_major, "RAID5_RAM_BUFF");
    cleanup();

    pr_info("brd module unloaded with no issues, goodjob.\n");
}


module_init(brd_init);
module_exit(brd_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RAM Disk Driver Implementation");
