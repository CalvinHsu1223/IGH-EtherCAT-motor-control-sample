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
#define FREQUENCY 1000
#define CLOCK_TO_USE CLOCK_REALTIME
#define MEASURE_TIMING 0
#define CONFIGURE_PDOS 1

// Optional features
#define PDO_SETTING1	1
#define PDO_SETTING2	1

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
static ec_domain_t *domain3 = NULL;
static ec_domain_state_t domain3_state = {};
static ec_domain_t *domain4 = NULL;
static ec_domain_state_t domain4_state = {};
/****************************************************************************/
static ec_slave_config_t *sc  = NULL;
static ec_slave_config_t *sc2 = NULL;
static ec_slave_config_state_t sc_state = {};
static ec_slave_config_state_t sc2_state = {};
/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;
static uint8_t *domain2_pd = NULL;
static uint8_t *domain3_pd = NULL;
static uint8_t *domain4_pd = NULL;
#define yas  		0,0
#define yas2 		1,0
#define yaskawa 	0x00000539, 0x02200001

/***************************** 20140224  ************************************/

//signal to turn off servo on state
static unsigned int servooff;
static unsigned int deactive;

// offsets for PDO entries
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

static signed long inpu[8]={};
static unsigned long change=0;
static unsigned long change2=0;

static signed 	long 	data[50000][3]={};
static unsigned long 	datacount=0;


//rx pdo entry of 1st motor
const static ec_pdo_entry_reg_t domain1_regs[] = {
   	{yas,  yaskawa,0x6040, 00,	&ctrl_word			},//rx
	{yas,  yaskawa,0x607a, 00,	&target_pos			},
	{yas,  yaskawa,0x60ff, 00,	&tar_velo			},
	{yas,  yaskawa,0x6071, 00,	&tar_torq			},
	{yas,  yaskawa,0x6072, 00,	&max_torq			},
	{yas,  yaskawa,0x6060, 00,	&modeofoper			},
	{yas,  yaskawa,0x60c1, 01,	&interpolateddata	},
	{}
};

//tx pdo entry of 1st motor
const static ec_pdo_entry_reg_t domain2_regs[] = {
    {yas,  yaskawa,0x6041, 00,	&status_word		},//tx
	{yas,  yaskawa,0x6064, 00,	&actual_pos			},
	{yas,  yaskawa,0x6077, 00,	&torq_actu_val		},
	{yas,  yaskawa,0x60f4, 00,	&following_actu_val	},
	{yas,  yaskawa,0x6061, 00,	&modeofop_display	},
	{yas,  yaskawa,0x60b9, 00,	&touch_probe_stat	},
	{yas,  yaskawa,0x60ba, 00,	&touch_probe_val	},
	{}
};

//rx pdo entry of 2nd motor
const static ec_pdo_entry_reg_t domain3_regs[] = {
   	{yas2,  yaskawa,0x6040, 00,	&ctrl_word2			},//rx
	{yas2,  yaskawa,0x607a, 00,	&target_pos2		},
	{yas2,  yaskawa,0x60ff, 00,	&tar_velo2			},
	{yas2,  yaskawa,0x6071, 00,	&tar_torq2			},
	{yas2,  yaskawa,0x6072, 00,	&max_torq2			},
	{yas2,  yaskawa,0x6060, 00,	&modeofoper2		},
	{yas2,  yaskawa,0x60c1, 01,	&interpolateddata2	},
	{}
};

//tx pdo entry of 2nd motor
const static ec_pdo_entry_reg_t domain4_regs[] = {
    {yas2,  yaskawa,0x6041, 00,	&status_word2		},//tx
	{yas2,  yaskawa,0x6064, 00,	&actual_pos2		},
	{yas2,  yaskawa,0x6077, 00,	&torq_actu_val2		},
	{yas2,  yaskawa,0x60f4, 00,	&following_actu_val2},
	{yas2,  yaskawa,0x6061, 00,	&modeofop_display2	},
	{yas2,  yaskawa,0x60b9, 00,	&touch_probe_stat2	},
	{yas2,  yaskawa,0x60ba, 00,	&touch_probe_val2	},
	{}
};


static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};
float i=0;
int j=0;

/*****************************************************************************/

#if PDO_SETTING1

		//yaskawa 1st PDO mapping
	static ec_pdo_entry_info_t slave_0_pdo_entries[] = {
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
	
#endif

/*****************************************************************************/
#if PDO_SETTING2

		//yaskawa 2nd PDO mapping
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
	
#endif

/*****************************************************************************/


/*****************************************************************************/

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

    domain1_state = ds2;
}

/*****************************************************************************/

void check_domain3_state(void)
{
    ec_domain_state_t ds3;
    ecrt_domain_state(domain3, &ds3);

	if (ds3.working_counter != domain3_state.working_counter)
		printf("domain3: WC %u.\n", ds3.working_counter);
	if (ds3.wc_state != domain3_state.wc_state)
        printf("domain3: State %u.\n", ds3.wc_state);

    domain3_state = ds3;
}

/*****************************************************************************/

void check_domain4_state(void)
{
    ec_domain_state_t ds4;
    ecrt_domain_state(domain4, &ds4);

	if (ds4.working_counter != domain4_state.working_counter)
		printf("domain4: WC %u.\n", ds4.working_counter);
	if (ds4.wc_state != domain4_state.wc_state)
        printf("domain4: State %u.\n", ds4.wc_state);

    domain4_state = ds4;
}

/*****************************************************************************/



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
#ifdef MEASURE_TIMING
    struct timespec startTime, endTime, lastStartTime = {};
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
             latency_min_ns = 0, latency_max_ns = 0,
             period_min_ns = 0, period_max_ns = 0,
             exec_min_ns = 0, exec_max_ns = 0;
#endif

    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);

	while(1) {

		if(deactive==1){
		break;
		}

		wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

        if (latency_ns > latency_max_ns) {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns) {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns) {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns) {
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns) {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns) {
            exec_min_ns = exec_ns;
        }
#endif

		
		//ecrt_master_send(master);
		
		// receive process data
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);
		ecrt_domain_process(domain2);
		ecrt_domain_process(domain3);
		ecrt_domain_process(domain4);		
		
		
		
		inpu[0]=EC_READ_U16(domain2_pd + status_word);
		inpu[1]=EC_READ_U32(domain2_pd + actual_pos);	
		inpu[2]=EC_READ_U16(domain4_pd + status_word2);
		inpu[3]=EC_READ_U32(domain4_pd + actual_pos2);
		
		//printf("\rmotor1_degree = %4f \t motor2_degree = %4f",((float)inpu[1]/1000) , ((float)inpu[3]/1000) );
		printf("\r%6f    \t    %6f    \t    %6f    \r ",((float)inpu[1]/1000) , ((float)inpu[3]/1000) , (((float)inpu[3]-(float)inpu[1])/1000));
		
		// check process data state (optional)
		//check_domain1_state();
		
				
		
		if (counter) {
			counter--;
		} else { // do this at 1 Hz
			counter = FREQUENCY;

			// check for master state (optional)
			//check_master_state();

#ifdef MEASURE_TIMING
            
			//output timing stats
			/*
            printf("period     %10u ... %10u\n",
                    period_min_ns, period_max_ns);
            printf("exec       %10u ... %10u\n",
                    exec_min_ns, exec_max_ns);
            printf("latency    %10u ... %10u\n",
                    latency_min_ns, latency_max_ns);
			*/

			//printf("PERIOD_NS = %d \t NSEC_PER_SEC = %d \n" ,PERIOD_NS ,NSEC_PER_SEC );
			//printf("in = %d\n",in);
				
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
#endif

			// calculate new process data
			blink = !blink;
			/*if(i<30){//5000
			i=i+1;
			j=j+1;
			}
			else{
			i=10;
			j=10;
			}
			*/							
		}
		
		if(i<=6.28)
		i=i+0.000628;
		else
		i=0;
		
		
		
		
		//clock_gettime(CLOCK_TO_USE, &time_write1);
		// write process data
		if(servooff==1){//servo off
		EC_WRITE_U16(domain1_pd+ctrl_word, 0x0006 );
		//deactive++;
		}	
		else if( (inpu[0]&0x0040) == 0x0040  ){
		EC_WRITE_U16(domain1_pd+ctrl_word, 0x0006 );
		}
		else if( (inpu[0]&0x006f) == 0x0021){
		EC_WRITE_U16(domain1_pd+ctrl_word, 0x0007 );
		}
		else if( (inpu[0]&0x027f) == 0x0233){
		EC_WRITE_U16(domain1_pd+ctrl_word, 0x000f);
		EC_WRITE_S32(domain1_pd+interpolateddata, 0);
		EC_WRITE_S32(domain1_pd+tar_velo, 0xfffff);
		EC_WRITE_S32(domain1_pd+max_torq, 0xf00);
		EC_WRITE_S32(domain1_pd+modeofoper, 7);
		}
		else if( (inpu[0]&0x027f) == 0x0237){//600 800
			if(change >= 0 && change<2  ){
				if(i<0.0125)
				EC_WRITE_S32(domain1_pd+interpolateddata, 0 );
				else 
				EC_WRITE_S32(domain1_pd+interpolateddata, (sin(i)*180000) );
				
			EC_WRITE_U16(domain1_pd+ctrl_word, 0x001f);
			change++;
			
			data[datacount][0]=(i*57.2928);
			data[datacount][1]=inpu[1];
			data[datacount][2]=inpu[3];
			datacount++;
			
			}
			else
			change = 0;
		}
	
	
		if(servooff==1){//servo off
		EC_WRITE_U16(domain3_pd+ctrl_word2, 0x0006 );
		deactive++ ;
		}	
		else if( (inpu[2]&0x0040) == 0x0040  )
		EC_WRITE_U16(domain3_pd+ctrl_word2, 0x0006 );
		
		else if( (inpu[2]&0x006f) == 0x0021)
		EC_WRITE_U16(domain3_pd+ctrl_word2, 0x0007 );
		
		
		else if( (inpu[2]&0x027f) == 0x0233){
		EC_WRITE_U16(domain3_pd+ctrl_word2, 0x000f);
		EC_WRITE_S32(domain3_pd+interpolateddata2, 0);
		EC_WRITE_S32(domain3_pd+tar_velo2, 0xfffff);
		EC_WRITE_S32(domain3_pd+max_torq2, 0xf00);
		EC_WRITE_S32(domain3_pd+modeofoper2, 7);
		
		}
		else if( (inpu[2]&0x027f) == 0x0237){//600 800
			if(change2 >= 0 && change2 <2  ){
				if(i<0.0125)
				EC_WRITE_S32(domain3_pd+interpolateddata2, 0 );
				else 
				EC_WRITE_S32(domain3_pd+interpolateddata2, (sin(i)*(180000)) );
				
			EC_WRITE_U16(domain3_pd+ctrl_word2, 0x001f);
			change2++;
			
			}
			else{
			change2 = 0;
			}
		}
		
		
		
		// write application time to master
		clock_gettime(CLOCK_TO_USE, &time);
		ecrt_master_application_time(master, TIMESPEC2NS(time));

		
		if (sync_ref_counter) {
			sync_ref_counter--;
		} 
		else {
			sync_ref_counter = 3; // sync every cycle
			ecrt_master_sync_reference_clock(master);
		}
		ecrt_master_sync_slave_clocks(master);

		
		//clock_gettime(CLOCK_TO_USE, &time_send1);
		
		// send process data
		ecrt_domain_queue(domain1);
		ecrt_domain_queue(domain2);
		ecrt_domain_queue(domain3);
		ecrt_domain_queue(domain4);
		
		ecrt_master_send(master);

		//clock_gettime(CLOCK_TO_USE, &time_send2);
		
			/*
			printf("***************************\n");
			printf("RECE: %10u.%9u\n      %10u.%9u\t%10u\n",time_rece1.tv_sec,time_rece1.tv_nsec,time_rece2.tv_sec,time_rece2.tv_nsec,DIFF_NS(time_rece1, time_rece2));
			printf("WRITE:%10u.%9u\n      %10u.%9u\t%10u\n",time_write1.tv_sec,time_write1.tv_nsec,time_write2.tv_sec,time_write2.tv_nsec,DIFF_NS(time_write1, time_write2));
			printf("SEND: %10u.%9u\n      %10u.%9u\t%10u\n",time_send1.tv_sec,time_send1.tv_nsec,time_send2.tv_sec,time_send2.tv_nsec,DIFF_NS(time_send1, time_send2));
			*/
			
			//printf("%10u\n",DIFF_NS(time_rece1, time_rece2));
			//printf("%10u\n",DIFF_NS(time_write1, time_write2));
			//printf("%10u\n",DIFF_NS(time_send1, time_send2));
			
		
#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
#endif
	
		

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
			
	domain3 = ecrt_master_create_domain(master);
    if (!domain3)
	        return -1;

	domain4 = ecrt_master_create_domain(master);
    if (!domain4)
	        return -1;		
			
	 	
	    
	if (!(sc = ecrt_master_slave_config(
                    master, yas, yaskawa))) {
		fprintf(stderr, "Failed to get slave1 configuration.\n");
        return -1;
    }
	if (!(sc2 = ecrt_master_slave_config(
                    master, yas2, yaskawa))) {
		fprintf(stderr, "Failed to get slave1 configuration.\n");
        return -1;
    }
		
#if CONFIGURE_PDOS
    printf("Configuring PDOs...\n");
	
    if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) {
        fprintf(stderr, "Failed to configure 1st PDOs.\n");
        return -1;
    }
	
	if (ecrt_slave_config_pdos(sc2, EC_END, slave_1_syncs)) {
        fprintf(stderr, "Failed to configure 2nd PDOs.\n");
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
		
	/*************************************************	motor2馬達domain註冊到domain_process data ************************************************/
	if (ecrt_domain_reg_pdo_entry_list(domain3, domain3_regs)) {
        fprintf(stderr, "2nd motor RX_PDO entry registration failed!\n");
        return -1;
    }
	if (ecrt_domain_reg_pdo_entry_list(domain4, domain4_regs)) {
        fprintf(stderr, "2nd motor TX_PDO entry registration failed!\n");
        return -1;
    }
	/*
	printf("************************************\n");	
	printf("* %d   %d   %d   %d   %d   %d   %d    *\n"	,ctrl_word2  ,target_pos2	,tar_velo2	,tar_torq2
														,max_torq2	,modeofoper2	,touch_probe_func2	
			);
	printf("************************************\n");	
	printf("************************************\n");	
	printf("* %d   %d   %d   %d   %d   %d   %d    *\n"	,status_word2		,actual_pos2		,torq_actu_val2		,following_actu_val2
														,modeofop_display2	,touch_probe_stat2	,touch_probe_val2
			);
	printf("************************************\n");		
	*/
	
	/*************************************************************************************************************************************/
	
	


    // configure SYNC signals for this slave
	//ecrt_slave_config_dc(sc, 0x0300, 1000000 , 500000, 0, 0);  //參考用 搭配FREQ = 4000
	//ecrt_slave_config_dc(sc, 0x0300, 1250000, 125000, 0, 0);  
	//ecrt_slave_config_dc(sc2, 0x0300, 1250000, 125000, 0, 0);  
	
	ecrt_slave_config_dc(sc, 	0x0300, 4000000, 0, 0, 0);  
	ecrt_slave_config_dc(sc2, 	0x0300, 4000000, 0, 0, 0); 

		
	/*  Slave configuration,
		AssignActivate word,
		SYNC0 cycle time [ns],
		SYNC0 shift time [ns] 
		SYNC1 cycle time [ns]
		SYNC1 shift time [ns].
	*/

    printf("Activating master...\n");
	
    if (ecrt_master_activate(master))
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }
	if (!(domain2_pd = ecrt_domain_data(domain2))) {
        return -1;
    }
	if (!(domain3_pd = ecrt_domain_data(domain3))) {
        return -1;
    }
	if (!(domain4_pd = ecrt_domain_data(domain4))) {
        return -1;
    }
	
	
	
	
	
    pid_t pid = getpid();
    if (setpriority(PRIO_PROCESS, pid, -20))
        fprintf(stderr, "Warning: Failed to set priority: %s\n",
                strerror(errno));

				
	signal( SIGINT , endsignal ); //按CTRL+C 利用中斷結束程式			
	printf("Starting cyclic function.\n");
    cyclic_task();
	printf("\nEnding   cyclic function.\n");
	printf("\nDeactive DC\n");
	//ecrt_slave_config_dc(sc	, 0, 1000000, 125000, 0, 0);  
	//ecrt_slave_config_dc(sc2, 0, 1000000, 125000, 0, 0);  
	printf("then Release Master\n");
	ecrt_release_master(master);
	//sleep(1);
	
	
	FILE *fp;
    fp = fopen("SIN_DC_data_2","w");     /* open file pointer */
   
	for ( j=0;i<datacount;i++ )
    {
	fprintf(fp,"%8d\t%8d\t%8d\n"	,data[j][0]
									,data[j][1]
									,data[j][2]);
	}
    fclose(fp);
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
    return 0;
}
