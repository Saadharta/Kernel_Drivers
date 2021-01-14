#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h> 
#include <linux/uaccess.h>

#define LICENCE     "GPL"
#define AUTEUR      "FE Demiguel"
#define DESCRIPTION "driver attempt for DS18B20 thermal sensor"
#define DEVICE      "one_wire_device"

/* Custom consts */
#define MAX_DEV      1
#define SINGLE_SLAVE 1
#define GPIO_NUM     4
#define LABEL        "THERMAL"

/* OneWire Cmmand Set */
#define SEARCH_ROM   0xF0
#define READ_ROM     0x33
#define MATCH_ROM    0x55
#define SKIP_ROM     0xCC
#define SEARCH_ALARM 0xEC

/* Sensor Command Set */
#define CONVERT_INIT   0x44  // Tells device to initiate temperature conversion
#define COPY_SCRATCH   0x48  // Copy bytes 2 - 4 from scratchpad to EEPROM
#define WRITE_SCRATCH  0x4E  // Write bytes 2 - 4 of scratchpad /!\ master MUSTN'T RESSET before all 3 bytes are written
#define READ_SCRATCH   0xBE  // Read entire scratchpad /!\ master can interrupt at anytime via RESET
#define RECALL_SCRATCH 0xB8  // Recall bytes 2 - 4 from EEPROM to scratchpad

/* Sensor Scratchpad locations */
#define TEMP_LSB        0
#define TEMP_MSB        1
#define ALARM_HIGH      2
#define ALARM_LOW       3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

#define MAX_CONVERSION_TIMEOUT		750

/* custom vars */
char *deviceName="DS18B20";
char alarm_high= 0xC9;
char alarm_low= 0x7D;
int gpio_pin = GPIO_NUM;
module_param(gpio_pin, int, S_IRUGO);

/* Char driver functions */
static ssize_t therm_read(struct file *f, char *buf, size_t size, loff_t *offset);
static ssize_t therm_write(struct file *f, const char *buf, size_t size, loff_t *offset);
static int therm_open(struct inode *in, struct file *f);
static int therm_release(struct inode *in, struct file *f);

/* One Wire related functions */
static void therm_reset(void);
static void therm_write_byte(char cmd);
static char therm_read_byte(void);
static void therm_configure(char t_high, char t_low, char config);
static void therm_convert(void);
static int  therm_do_int(char lsb, char msb);
static int  therm_do_float(char lsb);
static void therm_read_scratch(void);
static void therm_get_address(void);

// standard file_ops for char driver
static struct file_operations therm_fops =
{
  .owner = THIS_MODULE,
  .read = therm_read,
  .write = therm_write,
  .open = therm_open,
  .release = therm_release,
};

/* Structure to store slaves data */
struct slave_address{
  //struct list_head lslv;
  char addr[8];
  char scratch[9];
};

struct slave_address slv;
/* The dev_t for our driver */
dev_t dev;
/* The cdev structure for our devices */
struct cdev *my_cdev;
/* Classy way to nullify the need for an explicit mknod */
static struct class *my_class;

static DEFINE_MUTEX(my_mutex);

static ssize_t therm_read(struct file *f, char *buf, size_t size,loff_t *offset){
  int ip,fp, fl_size;
  int ret;
  char pseudo_float[16];
  printk(KERN_NOTICE "%s : read start\n", deviceName);
  therm_convert();
  therm_read_scratch();
  ip = therm_do_int(slv.scratch[TEMP_LSB], slv.scratch[TEMP_MSB]);
  fp = therm_do_float(slv.scratch[TEMP_LSB]);
  if(ip&0x80){ //gestion du signe
    sprintf(pseudo_float, "-%d.%04d",ip, fp);
  }else{
    sprintf(pseudo_float, "%d.%04d",ip, fp);
  }
  fl_size = strlen(pseudo_float);
  if(copy_to_user(buf, pseudo_float, fl_size)==0){
    ret = fl_size;
  }else{
    ret = -EFAULT;
  }
  printk(KERN_NOTICE "%s : read done\n", deviceName);
	return ret;
}

static ssize_t therm_write(struct file *f, const char *buf, size_t size,loff_t *offset){
  char *read_buff;
  int new_res, err = 0;
  printk(KERN_NOTICE "%s : read start\n", deviceName);
  if( size ){
    read_buff = kcalloc(size, sizeof(char), GFP_KERNEL);
    if( read_buff ){
      err = copy_from_user(read_buff, buf, size);
      if(!err){
        err = kstrtoint(read_buff, 10, &new_res);
        if(!err){
          printk(KERN_NOTICE "%s : new resolution is %d\n", deviceName, new_res);
          therm_read_scratch();
          switch(new_res){
            case 9:
              therm_configure(slv.scratch[ALARM_HIGH],slv.scratch[ALARM_LOW], TEMP_9_BIT);
              break;
            case 10:
              therm_configure(slv.scratch[ALARM_HIGH],slv.scratch[ALARM_LOW], TEMP_10_BIT);
              break;
            case 11:
              therm_configure(slv.scratch[ALARM_HIGH],slv.scratch[ALARM_LOW], TEMP_11_BIT);
              break;
            case 12:
              therm_configure(slv.scratch[ALARM_HIGH],slv.scratch[ALARM_LOW], TEMP_12_BIT);
              break;
            default:
              printk(KERN_ALERT "%s : unknown resolution since default case has been reached\n", deviceName);
          }
          therm_convert();
        }else if(err == -ERANGE){
          printk(KERN_ALERT "%s : overflow while casting to hex", deviceName);
        }else if(err == -EINVAL) {
          printk(KERN_ALERT "%s : user resolution not defined", deviceName);
        }
      }
    }
    kfree(read_buff);
  }
  printk(KERN_NOTICE "%s : read done\n", deviceName);
	return err;
}

void therm_write_byte(char cmd){
  int i;
  printk(KERN_NOTICE "%s : [bus='0x%02x'] master >>> slave@%02x%02x%02x%02x%02x%02x\n", deviceName, cmd, slv.addr[6],slv.addr[5],slv.addr[4],slv.addr[3],slv.addr[2],slv.addr[1]);
  for (i=0; i < 8; ++i){
    gpio_direction_output(gpio_pin, 0);
    udelay(15);
    if(cmd & 0x01){
      gpio_direction_input(gpio_pin);
      udelay(45);
    }else{
      udelay(45);
      gpio_direction_input(gpio_pin);
      udelay(2); // recovery time between bits : 2 µs
    }
    cmd>>=1; 
  }
  return;
}

char therm_read_byte(){
  int i;
  char ans = 0x00;
  for (i = 0; i <8; ++i){
    ans >>=1;
    gpio_direction_output(gpio_pin, 0);
    udelay(5); // master put the but at 0 for > 1µs
    gpio_direction_input(gpio_pin);
    udelay(10);
    if (gpio_get_value(gpio_pin)){
      ans |= 0x80; 
    }
    // need to respect the 60µs
    udelay(60); 
  }
  printk(KERN_NOTICE "%s : [bus='0x%02x'] master <<< slave@%02x%02x%02x%02x%02x%02x\n", deviceName, ans, slv.addr[6],slv.addr[5],slv.addr[4],slv.addr[3],slv.addr[2],slv.addr[1]);
  return ans;
}

void therm_reset(){
  int max_wait = 24;
  gpio_direction_output(gpio_pin,0);
  usleep_range(480, 500);
  gpio_direction_input(gpio_pin);
  usleep_range(15, 20);
  while(!gpio_get_value(gpio_pin) && !max_wait){
    udelay(10);
    printk(KERN_NOTICE "%s : waiting for slave, iter %d\n", deviceName, max_wait);
    --max_wait;
  }
  if(!max_wait)
    printk(KERN_ALERT "%s : failed to reset the bus\n", deviceName);
  udelay(120);
  return;
}

void therm_configure(char t_high, char t_low, char config){
  therm_reset();
  therm_write_byte(SKIP_ROM);
  therm_write_byte(WRITE_SCRATCH);
  therm_write_byte(t_high);
  therm_write_byte(t_low);
  therm_write_byte(config);
}

void therm_convert(){
  therm_reset();
  therm_write_byte(SKIP_ROM);
  therm_write_byte(CONVERT_INIT); 
  mdelay(MAX_CONVERSION_TIMEOUT);
  return;
}

int therm_do_int(char lsb, char msb){
  char ip = 0x00;
  ip = (lsb & 0xF0)>>4;
  ip |= (msb & 0x0F)<<4;
  return (int)ip;
}

int therm_do_float(char lsb){
  char res = slv.scratch[CONFIGURATION];
  int fp = 0;
  // the resolution means we need to obfuscate some bits
  if(res == TEMP_9_BIT){
    lsb &= 0xF8;
  }else if(res == TEMP_10_BIT){
    lsb &= 0xFC;
  }else if(res == TEMP_11_BIT){
    lsb &= 0xFE;
  }
  // 12 bits temps means we take everything, no need to occult decimal bits
  if(lsb & 0x08)
    fp += 5000;
  if(lsb & 0x04)
    fp +=2500;
  if(lsb & 0x02)
    fp +=1250;
  if(lsb & 0x01)
    fp +=625;
  return fp;
}

void therm_read_scratch(){
  int i;
  therm_reset();
  therm_write_byte(SKIP_ROM);
  therm_write_byte(READ_SCRATCH);
  for(i=0; i<9; ++i){
    slv.scratch[i] = therm_read_byte();
  }
  return;
}

void therm_get_address(){
  int i = 8;
  printk(KERN_NOTICE "%s : fetching slave ROM", deviceName);
  if(SINGLE_SLAVE){
    therm_reset();
    therm_write_byte(READ_ROM);
    while(i){
      slv.addr[7-(--i)] = therm_read_byte();
    }
  }else{
    printk(KERN_ALERT "%s: muliple slaves address fetch not implemented yet!", deviceName);
  }
  return;
}

static int therm_open(struct inode *in, struct file *f ){
  /* lock the mutex */
    mutex_lock(&my_mutex);
    return 0;
}

static int therm_release(struct inode *in, struct file *f ){
  /* free the mutex */
  mutex_unlock(&my_mutex);
  return 0;
}

int therm_init(void)
{
  int err = 0;
  printk(KERN_NOTICE "%s : initialisation start\n", deviceName);

	if (alloc_chrdev_region(&dev,0,MAX_DEV,deviceName) == -1){
		printk(KERN_ALERT "%s : error in alloc_chrdev_region\n", deviceName);
    goto out_fp0; 
	}
  my_cdev = cdev_alloc();
  if (!my_cdev){
    printk(KERN_ALERT "%s : error in cdev_alloc\n", deviceName);
    goto out_fp1; 
  }
  my_cdev->ops = &therm_fops;
  my_cdev->owner = THIS_MODULE;
  my_class = class_create(THIS_MODULE, deviceName);
  if (my_class == NULL){
    printk(KERN_ALERT "%s : error in class creation\n", deviceName);
    goto out_fp2;
  }
  if(device_create(my_class, NULL, dev, NULL, DEVICE)== NULL){
    printk(KERN_ALERT "%s : error in device creation\n", deviceName);
    goto out_fp3;
  }
  if(cdev_add(my_cdev,dev,MAX_DEV) ){
    printk(KERN_ALERT "%s : error in char device addition\n", deviceName);
    goto out_fp4;
  }
  if(gpio_request(gpio_pin, LABEL)){
    printk(KERN_ALERT "%s : error in gpio %d request\n", deviceName, gpio_pin);
    goto out_fp4;
  }
  /* TODO prepare the slave_address struct with INIT_LIST_HEAD(&slv.list); */
  goto out_safe;
out_fp4: // 4th fail point reaction 
  device_destroy(my_class, dev);
out_fp3: // 3rd fail point reaction
  class_destroy(my_class);
out_fp2: // 2nd fail point reaction
  cdev_del(my_cdev);
out_fp1: // 1st fail point reaction
  unregister_chrdev_region(dev,1);
out_fp0: // basic fail point reaction
  err = -EINVAL;
out_safe:
  printk(KERN_NOTICE "%s : out_safe reached, err = %d\n", deviceName,err);
  /* TODO get slaves addresses */
  if(!err){
    printk(KERN_NOTICE "%s : therm_get_address\n", deviceName);
    therm_get_address();
    printk(KERN_NOTICE "%s : therm_configure\n", deviceName);
    therm_configure(alarm_high, alarm_low, TEMP_12_BIT);
    printk(KERN_NOTICE "%s : therm_convert\n", deviceName);
    therm_convert();
    printk(KERN_NOTICE "%s : therm_read_scratch\n", deviceName);
    therm_read_scratch();
    printk(KERN_NOTICE "%s : initialisation done\n", deviceName);
  }
	return(err);
}

static void therm_cleanup(void) {
  printk(KERN_NOTICE "%s : cleanup start\n", deviceName);
  gpio_free(gpio_pin);
  device_destroy(my_class, dev);
  class_destroy(my_class);
  cdev_del(my_cdev);
  unregister_chrdev_region(dev,MAX_DEV);
  printk(KERN_NOTICE "%s : cleanup done\n", deviceName);
}

MODULE_LICENSE(LICENCE);
MODULE_AUTHOR(AUTEUR);
MODULE_DESCRIPTION(DESCRIPTION);
MODULE_SUPPORTED_DEVICE(DEVICE);

module_init(therm_init);
module_exit(therm_cleanup);