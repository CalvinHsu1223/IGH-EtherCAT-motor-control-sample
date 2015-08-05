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

#define FREQUENCY 4000 // task frequency in Hz
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

static ec_domain_t *domain2 = NULL;
static ec_domain_state_t domain2_state = {};

// RTAI
static RT_TASK task;
static SEM master_sem;
static cycles_t t_last_cycle = 0, t_critical;

/*****************************************************************************/

// process data
static uint8_t *domain1_pd; // process data memory
static uint8_t *domain2_pd;

/*
#define DigOutSlavePos(X) 0, (1 + (X))
#define CounterSlavePos   0, 2

#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL2008 0x00000002, 0x07d83052
#define IDS_Counter     0x000012ad, 0x05de3052
*/

#define yas  		0,0																			
#define yas2 		1,0																			
#define yaskawa 	0x00000539, 0x02200001	

/*
static int off_dig_out[NUM_DIG_OUT];
static int off_counter_in;
static int off_counter_out;

static unsigned int counter = 0;
static unsigned int blink_counter = 0;
static unsigned int blink = 0;
static u32 counter_value = 0U;
*/


// offsets for PDO entries   //
static unsigned int ctrl_word			,ctrl_word2;			
static unsigned int target_pos	        ,target_pos2;	
static unsigned int tar_velo			,tar_velo2;			
static unsigned int tar_torq			,tar_torq2;			
static unsigned int max_torq			,max_torq2;			
static unsigned int modeofoper			,modeofoper2;			
static unsigned int interpolateddata    ,interpolateddata2;

static unsigned int status_word			,status_word2;			
static unsigned int actual_pos	        ,actual_pos2;	
static unsigned int torq_actu_val		,torq_actu_val2;		
static unsigned int following_actu_val	,following_actu_val2;	
static unsigned int modeofop_display	,modeofop_display2;	
static unsigned int touch_probe_stat	,touch_probe_stat2;	
static unsigned int touch_probe_val		,touch_probe_val2;	

static unsigned int alstat;

//somecounters   //
static signed long inpu[8]={0,0,0,0,0,0,0,0};
static unsigned long change=0;
int servooff;
int deactive;
int start,speedup,speeddown,stop;
//variables for get date   //
static float 	data[80000][4]={};
static unsigned long 	datacount=0;



//rx pdo entry of 1st 2nd motor  //
const static ec_pdo_entry_reg_t domain1_regs[] = {
   	{yas,  yaskawa,0x6040, 00,	&ctrl_word			},//rx
	{yas,  yaskawa,0x607a, 00,	&target_pos			},
	{yas,  yaskawa,0x60ff, 00,	&tar_velo			},
	{yas,  yaskawa,0x6071, 00,	&tar_torq			},
	{yas,  yaskawa,0x6072, 00,	&max_torq			},
	{yas,  yaskawa,0x6060, 00,	&modeofoper			},
	{yas,  yaskawa,0x60c1, 01,	&interpolateddata		},
//	{yas,  yaskawa,0x7000, 00,      &alstat				},
	{yas2,  yaskawa,0x6040, 00,	&ctrl_word2			},//rx
	{yas2,  yaskawa,0x607a, 00,	&target_pos2		},
	{yas2,  yaskawa,0x60ff, 00,	&tar_velo2			},
	{yas2,  yaskawa,0x6071, 00,	&tar_torq2			},
	{yas2,  yaskawa,0x6072, 00,	&max_torq2			},
	{yas2,  yaskawa,0x6060, 00,	&modeofoper2		},
	{yas2,  yaskawa,0x60c1, 01,	&interpolateddata2	},
	{}
};

//tx pdo entry of 1st 2nd motor  //
const static ec_pdo_entry_reg_t domain2_regs[] = {
    {yas,  yaskawa,0x6041, 00,	&status_word		},//tx
	{yas,  yaskawa,0x6064, 00,	&actual_pos			},
	{yas,  yaskawa,0x6077, 00,	&torq_actu_val		},
	{yas,  yaskawa,0x60f4, 00,	&following_actu_val	},
	{yas,  yaskawa,0x6061, 00,	&modeofop_display	},
	{yas,  yaskawa,0x60b9, 00,	&touch_probe_stat	},
	{yas,  yaskawa,0x60ba, 00,	&touch_probe_val	},
	{yas2,  yaskawa,0x6041, 00,	&status_word2		},//tx
	{yas2,  yaskawa,0x6064, 00,	&actual_pos2		},
	{yas2,  yaskawa,0x6077, 00,	&torq_actu_val2		},
	{yas2,  yaskawa,0x60f4, 00,	&following_actu_val2},
	{yas2,  yaskawa,0x6061, 00,	&modeofop_display2	},
	{yas2,  yaskawa,0x60b9, 00,	&touch_probe_stat2	},
	{yas2,  yaskawa,0x60ba, 00,	&touch_probe_val2	},
	{}
};




/*****************************************************************************/
	//yaskawa 1st PDO mapping  //
	static ec_pdo_entry_info_t slave_0_pdo_entries[] = {
		{0x6040, 0x00, 16},//RXPDO
		{0x607a, 0x00, 32},
		{0x60ff, 0x00, 32},
		{0x6071, 0x00, 16},
		{0x6072, 0x00, 16},
		{0x6060, 0x00, 8},
		{0x0000, 0x00, 8},/////////////////
		{0x60c1, 0x01, 32},
		{0x6041, 0x00, 16},//TXPDO
		{0x6064, 0x00, 32},
		{0x6077, 0x00, 16},
		{0x60f4, 0x00, 32},
		{0x6061, 0x00, 8},
		{0x0000, 0x00, 32},
		{0x60b9, 0x00, 16},
		{0x60ba, 0x00, 32},
		};//{index,subindex,lenth}

	static ec_pdo_info_t slave_0_pdos[] = {
		{0x1600, 8, slave_0_pdo_entries + 0},
		{0x1a00, 8, slave_0_pdo_entries + 8},
	};

	static ec_sync_info_t slave_0_syncs[] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_DISABLE},
		{3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
		{0xff}
	};
/*****************************************************************************/
/*****************************************************************************/
	//yaskawa 1st PDO mapping  //
	static ec_pdo_entry_info_t slave_1_pdo_entries[] = {
		{0x6040, 0x00, 16},//RXPDO
		{0x607a, 0x00, 32},
		{0x60ff, 0x00, 32},
		{0x6071, 0x00, 16},
		{0x6072, 0x00, 16},
		{0x6060, 0x00, 8},
		{0x0000, 0x00, 8},
		{0x60c1, 0x01, 32},
		{0x6041, 0x00, 16},//TXPDO
		{0x6064, 0x00, 32},
		{0x6077, 0x00, 16},
		{0x60f4, 0x00, 32},
		{0x6061, 0x00, 8},
		{0x0000, 0x00, 32},
		{0x60b9, 0x00, 16},
		{0x60ba, 0x00, 32},
		};//{index,subindex,lenth}

	static ec_pdo_info_t slave_1_pdos[] = {
		{0x1600, 8, slave_1_pdo_entries + 0},
		{0x1a00, 8, slave_1_pdo_entries + 8},
	};

	static ec_sync_info_t slave_1_syncs[] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, slave_1_pdos + 0, EC_WD_DISABLE},
		{3, EC_DIR_INPUT, 1, slave_1_pdos + 1, EC_WD_DISABLE},
		{0xff}
	};
/*****************************************************************************/


void check_domain1_state(void)
{
    ec_domain_state_t ds;

    rt_sem_wait(&master_sem);
    ecrt_domain_state(domain1, &ds);
    rt_sem_signal(&master_sem);

    if (ds.working_counter != domain1_state.working_counter)
        printk(KERN_INFO PFX "Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printk(KERN_INFO PFX "Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    rt_sem_wait(&master_sem);
    ecrt_master_state(master, &ms);
    rt_sem_signal(&master_sem);

    if (ms.slaves_responding != master_state.slaves_responding)
        printk(KERN_INFO PFX "%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printk(KERN_INFO PFX "AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printk(KERN_INFO PFX "Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

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
/*
	if ( (inpu[0]==0x0650) &&(stop==1)   )
	break;
*/



        // receive process data
        rt_sem_wait(&master_sem);
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
	ecrt_domain_process(domain2);
        rt_sem_signal(&master_sem);

        // check process data state (optional)
        //check_domain1_state();

		
		
		inpu[0]=EC_READ_U16(domain2_pd + status_word);
		inpu[1]=EC_READ_U32(domain2_pd + actual_pos);	
		inpu[2]=EC_READ_U16(domain2_pd + status_word2);
		inpu[3]=EC_READ_U32(domain2_pd + actual_pos2);
		/*
        if (counter) {
            counter--;
        } else {
            u32 c;

            counter = FREQUENCY;

            // check for master state (optional)
            check_master_state();

            c = EC_READ_U32(domain1_pd + off_counter_in);
            if (counter_value != c) {
                counter_value = c;
                printk(KERN_INFO PFX "counter=%u\n", counter_value);
            }
        }
		*/
		
		/*
        if (blink_counter) {
            blink_counter--;
        } else {
            blink_counter = 9;

            // calculate new process data
            blink = !blink;
        }
		*/
		
		
        // write process data
		/*
        for (i = 0; i < NUM_DIG_OUT; i++) {
            EC_WRITE_U8(domain1_pd + off_dig_out[i], blink ? 0x66 : 0x99);
        }

        EC_WRITE_U8(domain1_pd + off_counter_out, blink ? 0x00 : 0x02);
		*/
		
		
		
		
		
		
		
		
//		if(servooff==1){//servo off
		if(stop==1){
			
			if( ( inpu[0] == 0x1637 )  &&  ( inpu[2] == 0x1637 ) ){
			EC_WRITE_U16(domain1_pd+ctrl_word, 0x0006 );
            		EC_WRITE_U16(domain1_pd+ctrl_word2, 0x0006 );
			}
			else if( ( inpu[0] == 0x0650 )  &&  ( inpu[2] == 0x0650 ) ){
			//++deactive ;
			//EC_WRITE_U16(domain1_pd+alstat, 0x0002 );
			printk(KERN_INFO PFX "want to PREOP");
			deactive++;
			}
			/*
			else{
			EC_WRITE_U16(domain1_pd+alstat, 0x0002 );			
			printk(KERN_INFO PFX "want to PREOP");
			break;
			}
			*/
		
		}	
		else if( (inpu[0]&0x0040) == 0x0040  &&  (inpu[2]&0x0040) == 0x0040 ){
		EC_WRITE_U16(domain1_pd+ctrl_word, 0x0006 );
		EC_WRITE_U16(domain1_pd+ctrl_word2, 0x0006 );
		}
		else if( (inpu[0]&0x006f) == 0x0021 && (inpu[2]&0x006f) == 0x0021){
		EC_WRITE_U16(domain1_pd+ctrl_word, 0x0007 );
		EC_WRITE_U16(domain1_pd+ctrl_word2, 0x0007 );
		}
		else if( (inpu[0]&0x027f) == 0x0233 && (inpu[2]&0x027f) == 0x0233){
		EC_WRITE_U16(domain1_pd+ctrl_word, 0x000f);
		EC_WRITE_S32(domain1_pd+interpolateddata, 0);
		//EC_WRITE_S32(domain1_pd+tar_velo, 0xffffff);
		EC_WRITE_S32(domain1_pd+max_torq, 0xf00);
		EC_WRITE_S32(domain1_pd+modeofoper, 8);
		
		EC_WRITE_U16(domain1_pd+ctrl_word2, 0x000f);
		EC_WRITE_S32(domain1_pd+interpolateddata2, 0);
		//EC_WRITE_S32(domain1_pd+tar_velo2, 0xffffff);
		EC_WRITE_S32(domain1_pd+max_torq2, 0xf00);
		EC_WRITE_S32(domain1_pd+modeofoper2, 8);
		
		}
		else if( (inpu[0]&0x027f) == 0x0237  && (inpu[2]&0x027f) == 0x0237 ){
			//if(change >= 0 && change<2  ){
			if( change<1  ){	
				
				//start=1;

				//if(i==0){
				//EC_WRITE_S32(domain1_pd+interpolateddata	, 0 );
				//EC_WRITE_S32(domain1_pd+interpolateddata2	, 0 );
				//EC_WRITE_S32(domain1_pd+target_pos	, 0 );
				//EC_WRITE_S32(domain1_pd+target_pos2	, 0 );
				//EC_WRITE_S32(domain1_pd+tar_velo	, 0 );	
				//EC_WRITE_S32(domain1_pd+tar_velo2	, 0 );
				//}
				//else {
				//EC_WRITE_S32(domain1_pd+interpolateddata	, (sin(i)*180000) ); 	//for mode 7
				//EC_WRITE_S32(domain1_pd+interpolateddata2	, (sin(i)*180000) );
				//EC_WRITE_S32(domain1_pd+target_pos	, (sin(i)*180000) );		//for mode 8 with sin
				//EC_WRITE_S32(domain1_pd+target_pos2	, (sin(i)*180000) );
				EC_WRITE_S32(domain1_pd+target_pos      , inpu[7] );            //for mode 8 no sin
                                EC_WRITE_S32(domain1_pd+target_pos2     , inpu[7] );
				//EC_WRITE_S32(domain1_pd+tar_velo	, 500000 );		//for mode 9
				//EC_WRITE_S32(domain1_pd+tar_velo2	, 500000 );
					//if(1){
					
					
					if(inpu[7]==1800000){
					speedup=0;
					speeddown=1;
					//printk(KERN_INFO PFX "top");					
					inpu[7]=inpu[7]-200;
					}
					else if(speeddown==1 && inpu[7]!=0){
					inpu[7]=inpu[7]-200;
					//printk(KERN_INFO PFX "slow down");
					}
					else if(speeddown==1 && inpu[7]==0){
					speedup=0;
					speeddown=0;
					stop=1;
					//printk(KERN_INFO PFX "stop");
					}
					else if(!stop){
					speedup=1;
					speeddown=0;
					inpu[7]=inpu[7]+2000;
					//printk(KERN_INFO PFX "fast up ");

					}
				
/*
					if(speedup==1)
						inpu[7]+=500;
					else if(speeddown==1)
						inpu[7]-=1000;
					else{
						inpu[7]=0;
						servooff=1;
					}
*/
					//EC_WRITE_S32(domain1_pd+tar_velo	, inpu[7] );		//for mode 9
					//EC_WRITE_S32(domain1_pd+tar_velo2	, inpu[7] );
					//}
					//else{
					//EC_WRITE_S32(domain1_pd+tar_velo	, inpu[7] );		//for mode 9
					//EC_WRITE_S32(domain1_pd+tar_velo2	, inpu[7] );
					//}
				
				//}
				
				EC_WRITE_U16(domain1_pd+ctrl_word, 0x001f);
				EC_WRITE_U16(domain1_pd+ctrl_word2, 0x001f);
				change++;
				
				/*
				if(datacount<10001){
				data[datacount][0]=(sin(i)*360000);
				data[datacount][1]=inpu[1];
				data[datacount][2]=inpu[3];
				data[datacount][3]=(inpu[1] - inpu[3]);
				datacount++;
				}
				*/
			}
			else
			change = 0;
		}
		
		
		
		
		
		
		
	//printk(KERN_INFO PFX "pos1=%d   pos2=%d   inpu7=%d\n",inpu[1],inpu[3],inpu[7]);

		
		
		
        rt_sem_wait(&master_sem);

        tv.tv_usec += 100;
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
	ecrt_domain_queue(domain2);
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
	ec_slave_config_t *sc2;

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
	if (!(domain2 = ecrt_master_create_domain(master))) {
        printk(KERN_ERR PFX "Domain2 creation failed!\n");
        goto out_release_master;
    }

    printk(KERN_INFO PFX "Configuring PDOs...\n");

    // create configuration for reference clock FIXME
    if (!(sc = ecrt_master_slave_config(master, yas, yaskawa))) {
        printk(KERN_ERR PFX "Failed to get slave configuration.\n");
        goto out_release_master;
    }
	if (!(sc2 = ecrt_master_slave_config(master, yas2, yaskawa))) {
        printk(KERN_ERR PFX "Failed to get slave configuration.\n");
        goto out_release_master;
    }
/*
    for (i = 0; i < NUM_DIG_OUT; i++) {
        if (!(sc = ecrt_master_slave_config(master,
                        DigOutSlavePos(i), Beckhoff_EL2008))) {
            printk(KERN_ERR PFX "Failed to get slave configuration.\n");
            goto out_release_master;
        }

        if (ecrt_slave_config_pdos(sc, EC_END, el2008_syncs)) {
            printk(KERN_ERR PFX "Failed to configure PDOs.\n");
            goto out_release_master;
        }

        off_dig_out[i] = ecrt_slave_config_reg_pdo_entry(sc,
                0x7000, 1, domain1, NULL);

        if (off_dig_out[i] < 0)
            goto out_release_master;
    }
*/

/*
    if (!(sc = ecrt_master_slave_config(master,
                    CounterSlavePos, IDS_Counter))) {
        printk(KERN_ERR PFX "Failed to get slave configuration.\n");
        goto out_release_master;
    }
    off_counter_in = ecrt_slave_config_reg_pdo_entry(sc,
            0x6020, 0x11, domain1, NULL);
    if (off_counter_in < 0)
        goto out_release_master;
    off_counter_out = ecrt_slave_config_reg_pdo_entry(sc,
            0x7020, 1, domain1, NULL);
    if (off_counter_out < 0)
        goto out_release_master;
*/
		if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) {
            printk(KERN_ERR PFX "Failed to configure PDOs.\n");
            goto out_release_master;
        }
		if (ecrt_slave_config_pdos(sc2, EC_END, slave_1_syncs)) {
            printk(KERN_ERR PFX "Failed to configure PDOs.\n");
            goto out_release_master;
        }
		
		
		if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
		printk(KERN_ERR PFX "1st motor RX_PDO entry registration failed!\n");
        goto out_release_master;
		}	
	
		if (ecrt_domain_reg_pdo_entry_list(domain2, domain2_regs)) {
        printk(KERN_ERR PFX "1st motor RX_PDO entry registration failed!\n");
        goto out_release_master;
		}
		
		



    // configure SYNC signals for this slave
    //ecrt_slave_config_dc(sc, 0x0700, 1000000, 440000, 0, 0);
//	ecrt_slave_config_dc(sc, 	0x0300, 1000000, 125000, 0, 0);  
//	ecrt_slave_config_dc(sc2, 	0x0300, 1000000, 125000, 0, 0); 

    printk(KERN_INFO PFX "Activating master...\n");
    
	if (ecrt_master_activate(master)) {
        printk(KERN_ERR PFX "Failed to activate master!\n");
        goto out_release_master;
    }

    // Get internal process data for domain
    domain1_pd = ecrt_domain_data(domain1);
	domain2_pd = ecrt_domain_data(domain2);

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


	//ecrt_slave_config_dc(sc,        0x000, 2000000, 125000, 0, 0);
        //ecrt_slave_config_dc(sc2,       0x000, 2000000, 125000, 0, 0);

	//if( ( inpu[0] == 0x1637 )  &&  ( inpu[2] == 0x1637 ) ){
        //EC_WRITE_U16(domain1_pd+ctrl_word, 0x0002 );
        //EC_WRITE_U16(domain1_pd+ctrl_word2, 0x0002 );
        //}
        //else if( ( inpu[0] == 0x0650 )  &&  ( inpu[2] == 0x0650 ) ){
        //++deactive ;
        //}




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
