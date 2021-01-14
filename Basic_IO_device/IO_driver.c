/* Includes */
// maybe clean up some of them? 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>

#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/blkdev.h>
#include <linux/blk_types.h>
#include <linux/genhd.h>


#define LICENCE "GPL"
#define AUTEUR "FE D"
#define DESCRIPTION "My Block Device"
#define DEVICE "my_block_device"

#define DEF_MAJOR 0
#define BLOCK_MINORS 1
#define BLOCKNAME "my_block_device"
#define SECSIZE 1024            /* page4, block size 4ko*/
#define KERNEL_SECTOR_SIZE 512  /* page4, sector size 512o*/

/* Peripheral's structure */
static struct rb_device { 
    unsigned int size;              /* Size of the device (in sectors) */ 
    int major;
    spinlock_t lock;                /* For exclusive access to our request queue */
    u8 *data;                     
    struct request_queue *rb_queue; /* Our request queue */ 
    struct gendisk *rb_disk;        /* kernel's internal representation */ 
}b_dev;

/* Block driver functions */
static int rb_open(struct block_device *rb_dev, fmode_t mode);
static void rb_release(struct gendisk *rb_disk, fmode_t mode);
static int rb_getgeo(struct block_device *rb_dev, struct hd_geometry *geo);
int rb_ioctl(struct block_device *rb_dev, fmode_t mode, uint cmd, unsigned long arg);

static int create_gendisk(struct rb_device *rb_dev, int maj);
static int init_queue(struct rb_device *rb_dev);
static void delete_gendisk(struct rb_device *rb_dev);

static void rb_request(struct request_queue *q);
static int rb_transfer(struct request *req);
/* custom vars here */
char *name="blk_dev";
module_param(name, charp, S_IRUGO);

/* standard file_ops for block driver */
static struct block_device_operations rb_fops = {
    .owner = THIS_MODULE,
    .open = rb_open,
    .release = rb_release,
    .getgeo = rb_getgeo, 
    .ioctl = rb_ioctl
};

static int rb_open(struct block_device *rb_dev, fmode_t modes){
    /* TODO */
    return 0;
}

static void rb_release(struct gendisk *rb_disk, fmode_t mode ){
    /* TODO */ 
    return;
}

static int rb_getgeo(struct block_device *rb_dev, struct hd_geometry *geo){
    /* TODO */
    return 0;
}

int rb_ioctl(struct block_device *rb_dev, fmode_t mode, uint cmd, unsigned long arg){ 
    /* TODO */
    return 0;
}

void rb_request(struct request_queue *q){
    struct request *req;
    struct rb_device *rb_dev = q->queuedata;
    while ((req= blk_fetch_request(rb_dev->rb_queue)) !=NULL){
        __blk_end_request_all(req, rb_transfer(req)); 
    }
    return;
}

static int rb_transfer(struct request *req){
    struct req_iterator it;
    struct bio_vec bv;
    char *buffer;
    unsigned int num_sector, tot_sector; 
    int write;
    sector_t beg, size;
    tot_sector = 0;
    write = rq_data_dir(req);
    beg = blk_rq_pos(req);
    size = blk_rq_sectors(req);
    rq_for_each_segment(bv,req,it){
        buffer = page_address(bv.bv_page)+bv.bv_offset;
        if(bv.bv_len % KERNEL_SECTOR_SIZE)
            printk(KERN_ALERT "bio vector size %u is illegal\n",bv.bv_len % KERNEL_SECTOR_SIZE);
        num_sector = bv.bv_len / KERNEL_SECTOR_SIZE;
        tot_sector +=num_sector;
        if(write){
            memcpy(b_dev.data+(KERNEL_SECTOR_SIZE*it.iter.bi_sector),buffer,bv.bv_len*sizeof(char)); 
        }else{
            memcpy(buffer,b_dev.data+(KERNEL_SECTOR_SIZE*it.iter.bi_sector),bv.bv_len*sizeof(char)); 
        }
    }
    if(tot_sector != size)
            printk(KERN_NOTICE "Warning, %u != %lu", tot_sector, size);
    return 0;
}

int rb_init(void){
    int status;
    printk(KERN_ALERT "Hello %s !\n", name);
    status = register_blkdev(DEF_MAJOR, name);
    if(status < 0){
        printk(KERN_ERR "Unable to register %s\n",name);
        return -EBUSY;
    }
    b_dev.major = status;
    b_dev.data = kmalloc(SECSIZE*KERNEL_SECTOR_SIZE, GFP_KERNEL);
    status = init_queue(&b_dev);
    if(status < 0)
        return status;
    b_dev.size = SECSIZE;
    status = create_gendisk(&b_dev,b_dev.major);
    if(status < 0)
        printk(KERN_ALERT "gendisk KO %d", status);
        return status;
    return 0;
}

int init_queue(struct rb_device *rb_dev){
    spin_lock_init(&rb_dev->lock);
    rb_dev->rb_queue = blk_init_queue(rb_request,&rb_dev->lock);
    if(rb_dev->rb_queue == NULL)
        return -ENOMEM;
    rb_dev->rb_queue->queuedata = rb_dev;
    return 0;
}

int create_gendisk(struct rb_device *rb_dev, int maj){
    rb_dev->rb_disk=alloc_disk(BLOCK_MINORS);
    if(!rb_dev->rb_disk){
        printk(KERN_NOTICE "alloc_disk failed for %s\n",name);
        return -ENOMEM;
    }
    rb_dev->rb_disk->major = maj;
    rb_dev->rb_disk->first_minor = 0;
    rb_dev->rb_disk->fops = &rb_fops;
    rb_dev->rb_disk->queue = rb_dev->rb_queue;
    rb_dev->rb_disk->private_data = rb_dev;
    snprintf(rb_dev->rb_disk->disk_name, 32, BLOCKNAME);
    /* rb_disk init complete */
    set_capacity(rb_dev->rb_disk,rb_dev->size);
    add_disk(rb_dev->rb_disk);
    return 0;
}

static void delete_gendisk(struct rb_device *rb_dev){
    if(rb_dev->rb_disk){
        del_gendisk(rb_dev->rb_disk);
    }
    return;
}

static void rb_cleanup(void)
{
    delete_gendisk(&b_dev);
    put_disk(b_dev.rb_disk);
    if(b_dev.rb_queue)
        blk_cleanup_queue(b_dev.rb_queue);
    kfree(b_dev.data);
    unregister_blkdev(b_dev.major,name);
    printk(KERN_ALERT "Goodbye %s\n", name);
}

module_exit(rb_cleanup);
module_init(rb_init);

MODULE_LICENSE(LICENCE);
MODULE_AUTHOR(AUTEUR);
MODULE_DESCRIPTION(DESCRIPTION);
MODULE_SUPPORTED_DEVICE(DEVICE);
