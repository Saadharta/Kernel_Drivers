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

static unsigned char dscrc_table[] = {        
    0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,      
  157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,       
   35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,      
  190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,       
   70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,      
  219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,      
  101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,      
  248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,      
  140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,       
   17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,      
  175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,       
   50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,      
  202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,       
   87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,      
  233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,      
  116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

/* custom vars */
char *deviceName="DS18B20";
u8 alarm_high= 0xC9;
u8 alarm_low= 0x7D;

u8 ROM_NO[8];
int last_discr;
int last_fam_discr;
bool last_dev_flg;
u8 crc8;
/* Structure to store slaves data */
typedef struct {
  struct list_head lslv;
  int slvid; // will be MINOR number
  u8 addr[8];
  u8 scratch[9];
}slave_t;

slave_t slv;
/* The dev_t for our driver */
dev_t dev;
/* The cdev structure for our devices */
struct cdev *my_cdev;
/* Classy way to nullify the need for an explicit mknod */
static struct class *my_class;

static DEFINE_MUTEX(my_mutex);

int gpio_pin = GPIO_NUM;
module_param(gpio_pin, int, S_IRUGO);


/* Char driver functions */
static ssize_t therm_read(struct file *f, char *buf, size_t size, loff_t *offset);
static ssize_t therm_write(struct file *f, const char *buf, size_t size, loff_t *offset);
static int therm_open(struct inode *in, struct file *f);
static int therm_release(struct inode *in, struct file *f);

/* One Wire related functions */
static void ow_write_0b0(void);
static void ow_write_0b1(void);
static bool ow_read_0bx(void);
static void ow_write_byte(u8 cmd, slave_t *slv);
static u8 ow_read_byte(slave_t *slv);
static bool ow_reset(void);
static bool ow_search(void);

unsigned char do_crc8(u8 val);

/* ds18b20 related functions */
static void therm_configure(u8 t_high, u8 t_low, u8 config, slave_t *slv);
static void therm_convert(slave_t *slv);
static int  therm_do_int(u8 lsb, u8 msb);
static int  therm_do_float(u8 lsb, u8 res);
static void therm_read_scratch(slave_t *slv);
static void therm_match_rom(slave_t *slv);
static int therm_get_address(void);

static slave_t *therm_get_slave(int id);
static int therm_kill_slave(void);

// standard file_ops for char driver
static struct file_operations therm_fops =
{
  .owner = THIS_MODULE,
  .read = therm_read,
  .write = therm_write,
  .open = therm_open,
  .release = therm_release,
};

static ssize_t therm_read(struct file *f, char *buf, size_t size,loff_t *offset)
{
  int ip,fp, fl_size;
  int ret;
  char pseudo_float[16];
  slave_t *slave;
  slave = therm_get_slave(MINOR(f->f_inode->i_rdev));
  if(slave == NULL)
    goto out_null_slv;
  printk(KERN_NOTICE "%s : read start\n", deviceName);
  therm_convert(slave);
  therm_read_scratch(slave);
  ip = therm_do_int(slave->scratch[TEMP_LSB], slave->scratch[TEMP_MSB]);
  fp = therm_do_float(slave->scratch[TEMP_LSB], slave->scratch[CONFIGURATION]);
  printk(KERN_NOTICE "%s : [bus='0x%02x'] master <<< slave@%02x%02x%02x%02x%02x%02x\n", deviceName, slave->scratch[TEMP_LSB], slave->addr[6],slave->addr[5],slave->addr[4],slave->addr[3],slave->addr[2],slave->addr[1]);
  printk(KERN_NOTICE "%s : [bus='0x%02x'] master <<< slave@%02x%02x%02x%02x%02x%02x\n", deviceName, slave->scratch[TEMP_MSB], slave->addr[6],slave->addr[5],slave->addr[4],slave->addr[3],slave->addr[2],slave->addr[1]);
  if(ip&0x80){ //gestion du signe
    sprintf(pseudo_float, "-%d.%04d",ip, fp);
  }else{
    sprintf(pseudo_float, "%d.%04d",ip, fp);
  }
  fl_size = strlen(pseudo_float);
  if(copy_to_user(buf, pseudo_float, fl_size)==0){
    ret = fl_size;
    printk(KERN_NOTICE "%s : read done\n", deviceName);
    goto out_return;
  }
  out_null_slv :
    ret = -EFAULT;
  out_return:
	  return ret;
}

static ssize_t therm_write(struct file *f, const char *buf, size_t size,loff_t *offset)
{
  char *read_buff;
  int new_res, err = 0;
  slave_t *slave;
  printk(KERN_NOTICE "%s : read start\n", deviceName);
  if( size ){
    slave = therm_get_slave(MINOR(f->f_inode->i_rdev));
    if(slave == NULL)
      goto out_null;   
    if( !(read_buff = kcalloc(size, sizeof(char), GFP_KERNEL)) )
      goto out_null;
    if( (err = copy_from_user(read_buff, buf, size)) )
      goto out_kfree;
    if(! (err = kstrtoint(read_buff, 10, &new_res)) )
      goto out_kfree;
    printk(KERN_NOTICE "%s : new resolution is %d\n", deviceName, new_res);
    therm_read_scratch(slave);
    switch(new_res){
      case 9:
        therm_configure(slave->scratch[ALARM_HIGH],slave->scratch[ALARM_LOW], TEMP_9_BIT, slave);
        break;
      case 10:
        therm_configure(slave->scratch[ALARM_HIGH],slave->scratch[ALARM_LOW], TEMP_10_BIT, slave);
        break;
      case 11:
        therm_configure(slave->scratch[ALARM_HIGH],slave->scratch[ALARM_LOW], TEMP_11_BIT, slave);
        break;
      case 12:
        therm_configure(slave->scratch[ALARM_HIGH],slave->scratch[ALARM_LOW], TEMP_12_BIT, slave);
        break;
      default:
        printk(KERN_ALERT "%s : unknown resolution since default case has been reached\n", deviceName);
    }
    therm_convert(slave);
  }
  out_kfree:
    if(err == -ERANGE)
      printk(KERN_ALERT "%s : overflow while casting to hex", deviceName);
    if(err == -EINVAL) 
      printk(KERN_ALERT "%s : user resolution not defined", deviceName);
    kfree(read_buff);
  out_null:
    printk(KERN_NOTICE "%s : read done\n", deviceName);
	return err;
}

void ow_write_0b0(void)
{
  gpio_direction_output(gpio_pin, 0);
  udelay(60);
  gpio_direction_input(gpio_pin);
  udelay(2); // slave recovery time 
  return;
}

void ow_write_0b1(void)
{
  gpio_direction_output(gpio_pin, 0);
  udelay(15);
  gpio_direction_input(gpio_pin);
  udelay(45);
  return;
}

bool ow_read_0bx(void)
{
  bool c = 0;
  gpio_direction_output(gpio_pin, 0);
  udelay(5);
  gpio_direction_input(gpio_pin);
  udelay(10);
  c|= gpio_get_value(gpio_pin);
  udelay(60);
  return c;
}

void ow_write_byte(u8 cmd, slave_t *slv)
{
  int i; 
  if(slv == NULL)
    printk(KERN_NOTICE "%s : [bus='0x%02x'] master >>> slave@xxxxxxxxxxxx\n", deviceName, cmd);
  else
    printk(KERN_NOTICE "%s : [bus='0x%02x'] master >>> slave@%02x%02x%02x%02x%02x%02x\n", deviceName, cmd, slv->addr[6],slv->addr[5],slv->addr[4],slv->addr[3],slv->addr[2],slv->addr[1]);
  for (i=0; i < 8; ++i){
    (cmd & 0x01)?ow_write_0b1():ow_write_0b0();
    cmd>>=1; 
  }
  return;
}

u8 ow_read_byte(slave_t *slv)
{
  int i;
  u8 ans = 0x00;
  for (i = 0; i <8; ++i){
    ans >>=1;
    if(ow_read_0bx())
      ans |=0x80;
  }
  printk(KERN_NOTICE "%s : [bus='0x%02x'] master <<< slave@%02x%02x%02x%02x%02x%02x\n", deviceName, ans, slv->addr[6],slv->addr[5],slv->addr[4],slv->addr[3],slv->addr[2],slv->addr[1]);
  return ans;
}

/* reset and signal if there is at least one slave on the bus */
bool ow_reset(void)
{
  int max_wait = 24;
  bool is_slave;
  gpio_direction_output(gpio_pin,0);
  usleep_range(480, 500);
  gpio_direction_input(gpio_pin);
  usleep_range(15, 20);
  while(!gpio_get_value(gpio_pin) && !max_wait){
    udelay(10);
    printk(KERN_NOTICE "%s : waiting for slave, iter %d\n", deviceName, max_wait);
    --max_wait;
  }
  is_slave = (!max_wait)? false : true;
  if(!max_wait)
    printk(KERN_ALERT "%s : failed to reset the bus\n", deviceName);
  udelay(120);
  return is_slave;
}

bool ow_search(void)
{
  int id_bit_number, last_zero, rom_byte_number;
  bool id_bit, id_cmp, search_res;
  u8 rom_byte_mask, search_direction;
 
  id_bit_number = 1;
  rom_byte_mask = 1;
  rom_byte_number = 0;
  last_zero = 0;
  crc8=0;
  if(!last_dev_flg){
    if(!ow_reset()){
      goto out_no_search_res;
    }
    ow_write_byte(SEARCH_ROM, NULL);
    do{
      id_bit = ow_read_0bx();
      id_cmp = ow_read_0bx();
      if(id_bit & id_cmp){
        break; // no slave
      }else{
        if(id_bit ^ id_cmp){
          search_direction = id_bit;
        }else{
          if(id_bit_number < last_discr)
            search_direction = (ROM_NO[rom_byte_number] & rom_byte_mask) > 0;
          else
            search_direction = (id_bit_number == last_discr);
          if(!search_direction){
            last_zero = id_bit_number;
            if(last_zero < 9)
              last_fam_discr = last_zero;
          }
        }
        if(search_direction){
          ROM_NO[rom_byte_number] |= rom_byte_mask;
          ow_write_0b1();
        }else{
          ROM_NO[rom_byte_number] &= ~rom_byte_mask;
          ow_write_0b0();
        }
        ++id_bit_number;
        rom_byte_mask <<= 1;
        if(!rom_byte_mask){
          do_crc8(ROM_NO[rom_byte_number]);
          ++rom_byte_number;
          rom_byte_mask = 0x01;
          printk(KERN_NOTICE "%s : fcrc : %x, computed : %x\n", deviceName, ROM_NO[7], crc8);
        }
      }
    }while(rom_byte_number < 8);
    if(!((id_bit_number < 65) || (crc8 != 0))){
      last_discr = last_zero;
      if(!last_discr)
        last_dev_flg = true;
      search_res = true;
    }
  }
  if((!search_res) || (!ROM_NO[0]))
    goto out_res;

  out_no_search_res:
    last_discr = 0;
    last_dev_flg = false;
    last_fam_discr = 0;
    search_res = false;    
  out_res:
    return search_res;
}


unsigned char do_crc8(u8 val)
{
  crc8 = dscrc_table[crc8 ^ val];
  return crc8;
}

void therm_match_rom(slave_t *slv)
{
    int i;
    ow_write_byte(MATCH_ROM,slv);
    for (i = 0; i < 8; i++){
        ow_write_byte(slv->addr[i], slv);
    }
    return;
}

void therm_configure(u8 t_high, u8 t_low, u8 config, slave_t *slv)
{

  ow_reset();
  therm_match_rom(slv);
  ow_write_byte(WRITE_SCRATCH, slv);
  ow_write_byte(t_high, slv);
  ow_write_byte(t_low, slv);
  ow_write_byte(config, slv);
}

void therm_convert(slave_t *slv)
{
  ow_reset();
  therm_match_rom(slv);
  ow_write_byte(CONVERT_INIT, slv); 
  mdelay(MAX_CONVERSION_TIMEOUT);
  return;
}

int therm_do_int(u8 lsb, u8 msb)
{
  u8 ip = 0x00;
  ip = (lsb & 0xF0)>>4;
  ip |= (msb & 0x0F)<<4;
  return (int)ip;
}

int therm_do_float(u8 lsb, u8 res)
{
  int fp = 0;
  // the resolution means we need to obfuscate some bits
  if(res == TEMP_9_BIT){
    lsb &= 0xF8;
  }else if(res == TEMP_10_BIT){
    lsb &= 0xFC;
  }else if(res == TEMP_11_BIT){
    lsb &= 0xFE;
  } // 12 bits resolution means we take everything, no need to obfuscate decimal bits
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

void therm_read_scratch(slave_t *slv)
{
  int i;
  ow_reset();
  therm_match_rom(slv);
  ow_write_byte(READ_SCRATCH, slv);
  for(i=0; i<9; ++i){
    slv->scratch[i] = ow_read_byte(slv);
  }
  return;
}

int therm_get_address(void)
{
  int slv_cpt=0;
  bool toto;
  slave_t *slave;
  printk(KERN_NOTICE "%s : fetching slave ROM\n", deviceName);
  INIT_LIST_HEAD(&slv.lslv);
  
  do{
    toto = ow_search();
    slave = kmalloc(sizeof(slave_t), GFP_KERNEL);
    if(slave){
      list_add_tail(&slave->lslv, &(slv.lslv));
      slave->slvid = ++slv_cpt;
      memcpy(slave->addr, ROM_NO, sizeof(char)*8);
    }
  }while(toto);
  list_for_each_entry(slave, &slv.lslv, lslv){
    printk(KERN_NOTICE "%s : therm_configure\n", deviceName);
    therm_configure(alarm_high, alarm_low, TEMP_12_BIT, slave);
    printk(KERN_NOTICE "%s : therm_convert\n", deviceName);
    therm_convert(slave);
    printk(KERN_NOTICE "%s : therm_read_scratch\n", deviceName);
    therm_read_scratch(slave);
    printk(KERN_NOTICE "%s : slave %d found @%02x%02x%02x%02x%02x%02x\n", deviceName, slave->slvid, slave->addr[6],slave->addr[5],slave->addr[4],slave->addr[3],slave->addr[2],slave->addr[1]);
  }
  return slv_cpt;
}

static int therm_open(struct inode *in, struct file *f )
{
  /* lock the mutex */
    mutex_lock(&my_mutex);
    return 0;
}

static int therm_release(struct inode *in, struct file *f )
{
  /* free the mutex */
  mutex_unlock(&my_mutex);
  return 0;
}

slave_t *therm_get_slave(int id)
{
  slave_t *slave = NULL;
  list_for_each_entry(slave, &slv.lslv, lslv){
    if (slave->slvid == id)
      goto out_slv_search;
  }
  out_slv_search:
    return slave;
}

int therm_kill_slave(void){
  slave_t *pos, *q;
  int slv_amt = 0;
  list_for_each_entry_safe(pos, q, &slv.lslv, lslv){
    list_del(&(pos->lslv));
    kfree(pos);
    ++slv_amt;
  }
  return slv_amt;
}

int therm_init(void)
{
  slave_t *slave;
  int slv_amt,err = 0;
  printk(KERN_NOTICE "%s : initialisation start\n", deviceName);

  if(gpio_request(gpio_pin, LABEL)){
    printk(KERN_ALERT "%s : error in gpio %d request\n", deviceName, gpio_pin);
    goto out_fp0;
  }

  if(!ow_reset()){
    printk(KERN_ALERT "%s : no slaves detected; have a nice day\n", deviceName);
    goto out_fp0;
  }

  printk(KERN_NOTICE "%s : therm_get_address\n", deviceName);
  if(! (slv_amt = therm_get_address()) ){
    printk(KERN_ALERT "%s : something happened therefore no slaves were found after the reset\n", deviceName);
    therm_kill_slave();
    goto out_fp0;
  }

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
  list_for_each_entry(slave, &(slv.lslv), lslv){
    if(device_create(my_class, NULL, MKDEV(MAJOR(dev),MINOR(dev)+slave->slvid), slave, "%s%d", DEVICE, slave->slvid)== NULL){
      printk(KERN_ALERT "%s : error in device creation\n", deviceName);
      goto out_fp3;
    }
    printk(KERN_NOTICE "%s : device(%d,%d) created", deviceName, MAJOR(dev), MINOR(dev)+slave->slvid);
  }
  if(cdev_add(my_cdev,dev,slv_amt) ){
    printk(KERN_ALERT "%s : error in char device addition\n", deviceName);
    goto out_fp4;
  }
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
    printk(KERN_NOTICE "%s : initialisation done\n", deviceName);
  }
	return(err);
}

static void therm_cleanup(void) {
  int slv_amt;
  slv_amt = therm_kill_slave();
  printk(KERN_NOTICE "%s : cleanup start\n", deviceName);
  gpio_free(gpio_pin);
  do{
    device_destroy(my_class, MKDEV(MAJOR(dev), MINOR(slv_amt)) );
  }while(slv_amt--);
  class_destroy(my_class);
  cdev_del(my_cdev);
  unregister_chrdev_region(MKDEV(MAJOR(dev),0),slv_amt);
  printk(KERN_NOTICE "%s : cleanup done\n", deviceName);
}

MODULE_LICENSE(LICENCE);
MODULE_AUTHOR(AUTEUR);
MODULE_DESCRIPTION(DESCRIPTION);
MODULE_SUPPORTED_DEVICE(DEVICE);

module_init(therm_init);
module_exit(therm_cleanup);