/******************************************************************************
 *
 *  Distributed clocks sample for the IgH EtherCAT master.
 *
 *  $Id: dc_rtai_sample.c,v bc2d4bf9cbe5 2012/09/06 18:22:24 fp $
 *
 *  Copyright (C) 2006-2008  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

// Linux
#include <linux/module.h>
#include <linux/err.h>

// RTAI
#include <rtai_sched.h>
#include <rtai_sem.h>

// EtherCAT
#include "../../include/ecrt.h"

/*****************************************************************************/

// Module parameters

#define FREQUENCY 1000 // task frequency in Hz
#define INHIBIT_TIME 20

#define TIMERTICKS (1000000000 / FREQUENCY)

#define NUM_DIG_OUT 1

#define PFX "ec_dc_rtai_sample: "

/*****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

//static ec_domain_t *domain2 = NULL;
//static ec_domain_state_t domain2_state = {};

// RTAI
static RT_TASK task;
static SEM master_sem;
static cycles_t t_last_cycle = 0, t_critical;

/*****************************************************************************/

// process data
static uint8_t *domain1_pd; // process data memory
//static uint8_t *domain2_pd;

/*
#define DigOutSlavePos(X) 0, (1 + (X))
#define CounterSlavePos   0, 2

#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL2008 0x00000002, 0x07d83052
#define IDS_Counter     0x000012ad, 0x05de3052
*/
#define yas  		0,0																			
#define yaskawa 	0x0000066f, 0x525100d1


/*
static int off_dig_out[NUM_DIG_OUT];
static int off_counter_in;
static int off_counter_out;

static unsigned int counter = 0;
static unsigned int blink_counter = 0;
static unsigned int blink = 0;
static u32 counter_value = 0U;
*/


// offsets for PDO entries   


//signal to turn off servo on state
static unsigned int servooff =0;
static unsigned int deactive;
// offsets for PDO entries
static unsigned int ctrl_word  ;
static unsigned int mode ;
static unsigned int tar_torq    ;
static unsigned int max_torq    ;
static unsigned int tar_pos    ;
static unsigned int max_speed  ;
static unsigned int touch_probe_func;
static unsigned int tar_vel ;
static unsigned int error_code  ;
static unsigned int status_word;
static unsigned int mode_display ;
static unsigned int pos_act;
static unsigned int vel_act;
static unsigned int torq_act;
static unsigned int touch_probe_status;
static unsigned int touch_probe_pos;
static unsigned int digital_input;

static unsigned int counter = 0;

//somecounters   //
static signed long inpu[8]={};
static unsigned long change=0;
int start,speedup,speeddown,stop;
//variables for get date   //
static float 	data[80000][4]={};
static unsigned long 	datacount=0;
int value =0;


//rx pdo entry of 1st 2nd motor  //
const static ec_pdo_entry_reg_t domain1_regs[] = {
        {yas,  yaskawa,0x6040, 00,      &ctrl_word              },
        {yas,  yaskawa,0x6060, 00,      &mode                   },
        {yas,  yaskawa,0x6071, 00,      &tar_torq               },
        {yas,  yaskawa,0x6072, 00,      &max_torq               },
        {yas,  yaskawa,0x607a, 00,      &tar_pos                },
        {yas,  yaskawa,0x6080, 00,      &max_speed              },
        {yas,  yaskawa,0x60b8, 00,      &touch_probe_func       },
        {yas,  yaskawa,0x60ff, 00,      &tar_vel                },
        {yas,  yaskawa,0x603f, 00,      &error_code             },
        {yas,  yaskawa,0x6041, 00,      &status_word            },
        {yas,  yaskawa,0x6061, 00,      &mode_display           },
        {yas,  yaskawa,0x6064, 00,      &pos_act                },
        {yas,  yaskawa,0x606c, 00,      &vel_act                },
        {yas,  yaskawa,0x6077, 00,      &torq_act               },
        {yas,  yaskawa,0x60b9, 00,      &touch_probe_status     },
        {yas,  yaskawa,0x60ba, 00,      &touch_probe_pos        },
        {yas,  yaskawa,0x60fd, 00,      &digital_input          },
        {}
};



/*****************************************************************************/
	//yaskawa 1st PDO mapping  //
	static ec_pdo_entry_info_t slave_0_pdo_entries[] = {
        {0x6040, 0x00, 16},
        {0x6060, 0x00, 8 },
        {0x6071, 0x00, 16},
        {0x6072, 0x00, 16},
        {0x607a, 0x00, 32},
        {0x6080, 0x00, 32},
        {0x60b8, 0x00, 16},
        {0x60ff, 0x00, 32},
        {0x603f, 0x00, 16},
        {0x6041, 0x00, 16},
        {0x6061, 0x00, 8 },
        {0x6064, 0x00, 32},
        {0x606c, 0x00, 32},
        {0x6077, 0x00, 16},
        {0x60b9, 0x00, 16},
        {0x60ba, 0x00, 32},
        {0x60fd, 0x00, 32},
        };//{index,subindex,lenth}

	static ec_pdo_info_t slave_0_pdos[] = {
		{0x1600, 8, slave_0_pdo_entries + 0},
		{0x1a00, 9, slave_0_pdo_entries + 8},
	};

	static ec_sync_info_t slave_0_syncs[] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_DISABLE},
		{3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
		{0xff}
	};
/*****************************************************************************/


void run(long data)
{
    int i;
    struct timeval tv;
    unsigned int sync_ref_counter = 0;

    count2timeval(nano2count(rt_get_real_time_ns()), &tv);


//    while (deactive!=20) {
    while (1) {
        t_last_cycle = get_cycles();



        // receive process data
        rt_sem_wait(&master_sem);
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        rt_sem_signal(&master_sem);

        // check process data state (optional)
        //check_domain1_state();
				
		inpu[0]=EC_READ_U16(domain1_pd + status_word);
		inpu[1]=EC_READ_U32(domain1_pd + pos_act);	
		
//		if(servooff==1){//servo off
	//	if(stop==1){
			
	//		if( ( inpu[0] == 0x1637 )  &&  ( inpu[2] == 0x1637 ) ){
	//		EC_WRITE_U16(domain1_pd+ctrl_word, 0x0006 );
          //  		EC_WRITE_U16(domain1_pd+ctrl_word2, 0x0006 );
	//		}
	//		else if( ( inpu[0] == 0x0650 )  &&  ( inpu[2] == 0x0650 ) ){
	//		printk(KERN_INFO PFX "want to PREOP");
	//		deactive++;
	//		}
		
	//	}	

		if( (inpu[0]&0x004f) == 0x0040 ){
		EC_WRITE_U16(domain1_pd+ctrl_word, 0x0006 );
		}
		else if( (inpu[0]&0x006f) == 0x0021){
		EC_WRITE_U16(domain1_pd+ctrl_word, 0x0007 );
		}
		else if( (inpu[0]&0x006f) == 0x0023){
		EC_WRITE_U16(domain1_pd+ctrl_word, 0x000f);
		EC_WRITE_S32(domain1_pd+tar_pos, 0);
		EC_WRITE_S32(domain1_pd+max_torq, 0xf00);
		}
		else if( (inpu[0]&0x006f) == 0x0027){
                                EC_WRITE_U16(domain1_pd+ctrl_word, 0x001f);
				EC_WRITE_S32(domain1_pd+tar_pos      , value );            //for mode 8 no sin
					
					if(value==180000){
					speedup=0;
					speeddown=1;
					//printk(KERN_INFO PFX "top");					
					value=value-1;
					}
					else if(speeddown==1 && value!=0){
					value=value-1;
					//printk(KERN_INFO PFX "slow down");
					}
					else if(speeddown==1 && value==0){
					speedup=0;
					speeddown=0;
				//	stop=1;
					//printk(KERN_INFO PFX "stop");
					}
					else if(!stop){
					speedup=1;
					speeddown=0;
					value=value+1;
					//printk(KERN_INFO PFX "fast up ");

					}
				
				
//				change++;
				
//			}
//			else
//			change = 0;
		}
		
		
        rt_sem_wait(&master_sem);

        tv.tv_usec += 1000;
        if (tv.tv_usec >= 1000000)  {
            tv.tv_usec -= 1000000;
            tv.tv_sec++;
        }
        ecrt_master_application_time(master, EC_TIMEVAL2NANO(tv));

        if (sync_ref_counter) {
            sync_ref_counter--;
        } else {
            sync_ref_counter = 1; //original = 9
            ecrt_master_sync_reference_clock(master);
        }
		
        ecrt_master_sync_slave_clocks(master);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        rt_sem_signal(&master_sem);

        rt_task_wait_period();

	



    }
}

/*****************************************************************************/

void send_callback(void *cb_data)
{
    ec_master_t *m = (ec_master_t *) cb_data;

    // too close to the next real time cycle: deny access...
    if (get_cycles() - t_last_cycle <= t_critical) {
        rt_sem_wait(&master_sem);
        ecrt_master_send_ext(m);
        rt_sem_signal(&master_sem);
    }
}

/*****************************************************************************/

void receive_callback(void *cb_data)
{
    ec_master_t *m = (ec_master_t *) cb_data;

    // too close to the next real time cycle: deny access...
    if (get_cycles() - t_last_cycle <= t_critical) {
        rt_sem_wait(&master_sem);
        ecrt_master_receive(m);
        rt_sem_signal(&master_sem);
    }
}

/*****************************************************************************/

int __init init_mod(void)
{
    int ret = -1, i;
    RTIME tick_period, requested_ticks, now;
    ec_slave_config_t *sc;

    printk(KERN_INFO PFX "Starting...\n");

    rt_sem_init(&master_sem, 1);

    t_critical = cpu_khz * 1000 / FREQUENCY - cpu_khz * INHIBIT_TIME / 1000;

    master = ecrt_request_master(0);
    if (!master) {
        ret = -EBUSY;
        printk(KERN_ERR PFX "Requesting master 0 failed!\n");
        goto out_return;
    }

    ecrt_master_callbacks(master, send_callback, receive_callback, master);

    printk(KERN_INFO PFX "Registering domain...\n");
    if (!(domain1 = ecrt_master_create_domain(master))) {
        printk(KERN_ERR PFX "Domain creation failed!\n");
        goto out_release_master;
    }
//	if (!(domain2 = ecrt_master_create_domain(master))) {
//        printk(KERN_ERR PFX "Domain2 creation failed!\n");
//        goto out_release_master;
//    }

    printk(KERN_INFO PFX "Configuring PDOs...\n");

	if (!(sc = ecrt_master_slave_config(master, yas, yaskawa))) {
            printk(KERN_ERR PFX "Failed to get slave configuration.\n");
            goto out_release_master;
	}	
//	ecrt_slave_config_dc(sc, 0x300, 1000000, 0, 0, 0);
    	if (ecrt_slave_config_sdo8(sc, 0x6060, 0, 8)){
            printk(KERN_ERR PFX "Failed to configure SDOs.\n");
            goto out_release_master;
	}
        if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) {
            printk(KERN_ERR PFX "Failed to configure PDOs.\n");
            goto out_release_master;
        }

	if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
		printk(KERN_ERR PFX "1st motor RX_PDO entry registration failed!\n");
        	goto out_release_master;
	}	


    printk(KERN_INFO PFX "Activating master...\n");
    
	if (ecrt_master_activate(master)) {
        printk(KERN_ERR PFX "Failed to activate master!\n");
        goto out_release_master;
    }

    // Get internal process data for domain
    domain1_pd = ecrt_domain_data(domain1);
//	domain2_pd = ecrt_domain_data(domain2);

    printk(KERN_INFO PFX "Starting cyclic sample thread...\n");
    requested_ticks = nano2count(TIMERTICKS);
    tick_period = start_rt_timer(requested_ticks);
    printk(KERN_INFO PFX "RT timer started with %i/%i ticks.\n",
           (int) tick_period, (int) requested_ticks);

    if (rt_task_init(&task, run, 0, 2000, 0, 1, NULL)) {
        printk(KERN_ERR PFX "Failed to init RTAI task!\n");
        goto out_stop_timer;
    }

    now = rt_get_time();
    if (rt_task_make_periodic(&task, now + tick_period, tick_period)) {
        printk(KERN_ERR PFX "Failed to run RTAI task!\n");
        goto out_stop_task;
    }

    printk(KERN_INFO PFX "Initialized.\n");
    return 0;

 out_stop_task:
    rt_task_delete(&task);
 out_stop_timer:
    stop_rt_timer();
 out_release_master:
    printk(KERN_ERR PFX "Releasing master...\n");
    ecrt_release_master(master);
 out_return:
    rt_sem_delete(&master_sem);
    printk(KERN_ERR PFX "Failed to load. Aborting.\n");
    return ret;
}

/*****************************************************************************/

void __exit cleanup_mod(void)
{
    printk(KERN_INFO PFX "Stopping...\n");



    rt_task_delete(&task);
    stop_rt_timer();
    ecrt_release_master(master);
    rt_sem_delete(&master_sem);

    printk(KERN_INFO PFX "Unloading.\n");
}

/*****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Florian Pose <fp@igh-essen.com>");
MODULE_DESCRIPTION("EtherCAT distributed clocks sample module");

module_init(init_mod);
module_exit(cleanup_mod);

/*****************************************************************************/
