/*****************************************************************************
 *
 *  $Id: main.c,v bc2d4bf9cbe5 2012/09/06 18:22:24 fp $
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
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
 ****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>

/*
#include <stdlib.h>
#include <math.h>  
*/
/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY 4000
#define CLOCK_TO_USE CLOCK_REALTIME
#define CONFIGURE_PDOS 1

// Optional features
#define PDO_SETTING1	1
#define SDO_ACCESS      1


/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
	(B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};
static ec_domain_t *domain2 = NULL;
static ec_domain_state_t domain2_state = {};
/****************************************************************************/
static ec_slave_config_t *sc  = NULL;
static ec_slave_config_state_t sc_state = {};
/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;
static uint8_t *domain2_pd = NULL;
#define yas  		0,0
#define yaskawa 	0x0000066f, 0x525100d1

/***************************** 20140224  ************************************/

//signal to turn off servo on state
static unsigned int servooff =0;
static unsigned int deactive;
// offsets for PDO entries
static unsigned int ctrl_word   ;
static unsigned int mode  ;
static unsigned int tar_torq    ;
static unsigned int max_torq    ;
static unsigned int tar_pos    ;
static unsigned int max_speed  ;
static unsigned int touch_probe_func ;
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

static signed long temp[8]={};

//rx pdo entry of 1st motor
const static ec_pdo_entry_reg_t domain1_regs[] = {
        {yas,  yaskawa,0x6040, 00,      &ctrl_word              },
        {yas,  yaskawa,0x6060, 00,      &mode                   },
        {yas,  yaskawa,0x6071, 00,      &tar_torq               },
        {yas,  yaskawa,0x6072, 00,      &max_torq               },
        {yas,  yaskawa,0x607a, 00,      &tar_pos                },
        {yas,  yaskawa,0x6080, 00,      &max_speed              },
        {yas,  yaskawa,0x60b8, 00,      &touch_probe_func       },
        {yas,  yaskawa,0x60ff, 00,      &tar_vel                },
        {}
};
const static ec_pdo_entry_reg_t domain2_regs[] = {
        {yas,  yaskawa,0x603f, 00,      &error_code             },
        {yas,  yaskawa,0x6041, 00,      &status_word            },
        {yas,  yaskawa,0x6061, 00,      &mode_display           },
        {yas,  yaskawa,0x6064, 00,      &pos_act                },
        {yas,  yaskawa,0x606c, 00,      &vel_act                },
        {yas,  yaskawa,0x6077, 00,      &torq_act               },
        {yas,  yaskawa,0x60b9, 00,      &touch_probe_status     },
        {yas  ,yaskawa,0x60ba, 00,      &touch_probe_pos        },
        {yas,  yaskawa,0x60fd, 00,      &digital_input          },
        {}
};




float value = 0;
static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

/*****************************************************************************/

#if PDO_SETTING1

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
                {0, EC_DIR_OUTPUT, 0, NULL,EC_WD_DISABLE},
                {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_DISABLE},
                {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
                {0xff}
        };

#endif
/***********************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
	struct timespec result;

	if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
		result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
		result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
	} else {
		result.tv_sec = time1.tv_sec + time2.tv_sec;
		result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
	}

	return result;
}

/*****************************************************************************/

void endsignal(int sig)
{
	
	servooff = 1;
	signal( SIGINT , SIG_DFL );
}

/*****************************************************************************/


void check_domain1_state(void)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);

	//struct timespec time_wc1,time_wc2;
	if (ds.working_counter != domain1_state.working_counter)
		printf("Domain1: WC %u.\n", ds.working_counter);
	if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/*****************************************************************************/

void check_domain2_state(void)
{
    ec_domain_state_t ds2;
    ecrt_domain_state(domain2, &ds2);

	if (ds2.working_counter != domain2_state.working_counter)
		printf("Domain2: WC %u.\n", ds2.working_counter);
	if (ds2.wc_state != domain2_state.wc_state)
        	printf("Domain2: State %u.\n", ds2.wc_state);

    domain2_state = ds2;
}



void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

	if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/****************************************************************************/

void cyclic_task()
{
    struct timespec wakeupTime, time;
    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);

	while(1) {

		if(deactive==1){
		break;
		}

		wakeupTime = timespec_add(wakeupTime, cycletime);
     		clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

		
		// writter_receive(master);
 	   	ecrt_master_receive(master);
   		ecrt_domain_process(domain1);
   		ecrt_domain_process(domain2);

                temp[0]=EC_READ_U16(domain2_pd + status_word);
                temp[1]=EC_READ_S32(domain2_pd + mode_display);
//              temp[2]=EC_READ_U16(domain2_pd + pos_act);
//              temp[3]=EC_READ_U32(domain2_pd + vel_act);
//              temp[4]=EC_READ_S32(domain2_pd + torq_act);
//              temp[5]=EC_READ_U32(domain2_pd + digital_input);
                if (counter) {
                        counter--;
                } else { // do this at 1 Hz
                        counter = FREQUENCY;
			check_master_state();
                        blink = !blink;
                        }
                printf("after value =%x\n",temp[0]);
                // write process data
                if(servooff==1){//servo off
                EC_WRITE_U16(domain1_pd+ctrl_word, 0x0006 );
                printf("0 is ok\n");
                }
                else if( (temp[0]&0x004f) == 0x0040  ){
                EC_WRITE_U16(domain1_pd+ctrl_word, 0x0006 );
                printf("%x\n",temp[0]);
                printf("1 is ok\n");
                }
                else if( (temp[0]&0x006f) == 0x0021){
                EC_WRITE_U16(domain1_pd+ctrl_word, 0x0007 );
                printf("%x\n",temp[0]);
                printf("2 is ok\n");
                }
                else if( (temp[0]&0x006f) == 0x0023){
                EC_WRITE_U16(domain1_pd+ctrl_word, 0x000f );
                EC_WRITE_S32(domain1_pd+tar_pos,0);
                EC_WRITE_S32(domain1_pd+tar_vel, 0xffff);
                EC_WRITE_S32(domain1_pd+max_torq, 0xf00);
                printf("%x\n",temp[0]);
                printf("3 is ok\n");
                }
                else if( (temp[0]&0x006f) == 0x0027){
                        EC_WRITE_S32(domain1_pd+tar_pos, (value+=2000) );
                        EC_WRITE_U16(domain1_pd+ctrl_word, 0x001f);
                //      int q =0;
                //      for (q =0; q<6; q++)    
                //              printf("q = %x\n",temp[q]);
                        printf("4 is ok\n");
                }
		clock_gettime(CLOCK_TO_USE, &time);
		ecrt_master_application_time(master, TIMESPEC2NS(time));
		if (sync_ref_counter) {
			sync_ref_counter--;
		} 
		else {
			sync_ref_counter = 1; // sync every cycle
			ecrt_master_sync_reference_clock(master);
		}
		ecrt_master_sync_slave_clocks(master);

		
		
		// send process data
		ecrt_domain_queue(domain1);
		ecrt_domain_queue(domain2);
		
		ecrt_master_send(master);

		

	}
}

/****************************************************************************/

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;
	
	
	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		perror("mlockall failed");
		return -1;
	}
	   master = ecrt_request_master(0);
    if (!master)
	        return -1;

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
	        return -1;
	
    domain2 = ecrt_master_create_domain(master);
    if (!domain2)
	        return -1;
			
	 	
	    
	if (!(sc = ecrt_master_slave_config(
                    master, yas, yaskawa))) {
		fprintf(stderr, "Failed to get slave1 configuration.\n");
        return -1;
	}   
#if SDO_ACCESS
    if (ecrt_slave_config_sdo8(sc, 0x6060, 0, 8)){
        return -1;
    }
#endif

 
#if CONFIGURE_PDOS
    printf("Configuring PDOs...\n");
	
    if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) {
        fprintf(stderr, "Failed to configure 1st PDOs.\n");
        return -1;
    }
	

#endif

	/*************************************************	motor1馬達domain註冊到domain_process data **********************************************/
	if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "1st motor RX_PDO entry registration failed!\n");
        return -1;
    }	
	
	if (ecrt_domain_reg_pdo_entry_list(domain2, domain2_regs)) {
        fprintf(stderr, "1st motor TX_PDO entry registration failed!\n");
        return -1;
    }
		
	
	ecrt_slave_config_dc(sc, 	0x0300,500000 , 0, 0, 0);  

    printf("Activating master...\n");
	
    if (ecrt_master_activate(master))
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }
    if (!(domain2_pd = ecrt_domain_data(domain2))) {
        return -1;
    }
	
	
	
	
    pid_t pid = getpid();
    if (setpriority(PRIO_PROCESS, pid, -20))
        fprintf(stderr, "Warning: Failed to set priority: %s\n",
                strerror(errno));

	signal( SIGINT , endsignal ); //按CTRL+C 利用中斷結束程式			
	printf("Starting cyclic function.\n");
    	cyclic_task();
	ecrt_release_master(master);
	
    return 0;
}
