/* 
 * Example code for Simple Open EtherCAT master
 *
 * Usage : red2
 *
 * This is a redundancy test.
 *
 * (c)Arthur Ketels 2008
 * 
 * structura PDO o tinem in pdo_def.h
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include <inttypes.h>

#include "ethercat.h"
#include "pdo_def.h"

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500
#define EC_TIMEOUTRX 500

/*
 * 1000000 ns CYCLE_TIME
 *  500000 ns SHIFT_TIME ok
 *  600000 ns SHIFT_TIME ok
 *  700000 ns SHIFT_TIME ok
 *  400000 ns SHIFT_TIME ok
 * 
 * 2000000 ns CYCLE_TIME
 * 1000000 ns SHIFT_TIME ok
 * 
 * cu cat e mai mare CYCLE_TIME, cu atat sunt mai bune sansele ca sa mearga bine
 * 
 */
#define CYCLE_TIME 1000000 // ns
#define SHIFT_TIME 500000  // ns

#define SAMPLE_SIZE 1000 // size of the avg array

char ifname[32] = "enp2s0";
struct sched_param schedp;
char IOmap[4096];
pthread_t thread1, thread2;
struct timeval tv, t1, t2;
int dorun = 0;
int deltat, tmax = 0;
int64 toff, pid_integral, teth;
int DCdiff;
int os;
uint8 ob;
uint16 ob2;
uint8 *digout = 0;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP = FALSE;
uint8 currentgroup = 0;
struct timespec now_time, prev_time;
int64 ethercat_time = 0;
int64 prev_DCtime;
int64 delta_cycle, delta_shift, delta_position;
int64 delta_cycle_arr[SAMPLE_SIZE], delta_shift_arr[SAMPLE_SIZE], delta_position_arr[SAMPLE_SIZE];
uint32 delta_cycle_crt = 0, delta_shift_crt = 0, delta_position_crt = 0;
int64 delta_cycle_samples = 0, delta_shift_samples = 0, delta_position_samples = 0;
double delta_cycle_sum, delta_cycle_avg, delta_cycle_std;
double delta_shift_sum, delta_shift_avg, delta_shift_std;
double delta_position_sum, delta_position_avg, delta_position_std;
int32 target_position_sdo, target_position_absolute, position_increment, units_to_position;
int direction = 1;
int64 threshold_position_6065h;
int64 delta_position_abs;

// am incercat diverse variante, si ce avem acum pare a fi cel mai ok
// double Kp = 0.01, Ki = 0.05;     // SOEM
double Kp = 0.1, Ki = 0.0005;       // Raspberry Pi
// double Kp = 0.01, Ki = 0.005;


in_IS620N_t *pod_in1;
out_IS620N_t *pod_out1;

#define READ(slaveId, idx, sub, buf, comment)                                                                                                                        \
    {                                                                                                                                                                \
        buf = 0;                                                                                                                                                     \
        int __s = sizeof(buf);                                                                                                                                       \
        int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);                                                                                 \
        printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment); \
    }

#define WRITE(slaveId, idx, sub, buf, value, comment)                                                                                         \
    {                                                                                                                                         \
        int __s = sizeof(buf);                                                                                                                \
        buf = value;                                                                                                                          \
        int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);                                                          \
        printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment); \
    }

void redtest(void)
{
    int cnt, oloop, iloop, chk;
    uint16 current_state;
    // uint64 buf64;
    uint32 buf32;
    uint16 buf16;
    uint8 buf8;
    boolean hasdc;
    int inc;

    printf("Starting Redundant test\n");

    /* initialise SOEM */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            // suntem in init state?
            ec_readstate();
            printf("Slave %d State=0x%2.2x (should pe 0x01) StatusCode=0x%4.4x : %s\n", 1, ec_slave[1].state, ec_slave[1].ALstatuscode, ec_ALstatuscode2string(ec_slave[1].ALstatuscode));

            /*********************************************************************** 
             * INIT
             * pag 138 din System Advanced User Guide
             * SDO  No
             * RPDO No
             * TPDO No
             * Communication initialization
             ***********************************************************************/

            /* wait for all slaves to reach INIT state */
            printf("start EC_STATE_INIT request\n");
            ec_slave[0].state = EC_STATE_INIT;
            ec_writestate(0);
            chk = 200;
            inc = 0;
            do
            {
                current_state = ec_statecheck(0, EC_STATE_INIT, 10000 + inc * 5); inc++;
                printf(" state=0x%2.2x\n", current_state);
            } while (chk-- && (current_state != EC_STATE_INIT));
            printf("current state is 0x%2.2x and 0x%2.2x was requested\n", current_state, EC_STATE_INIT);

            /* unconfigure SYNC0 */
            ec_dcsync0(1, FALSE, 0, 0);
            printf("ec_dcsync0 unconfigure ok\n");

            /*********************************************************************** 
             * PRE_OP
             * SDO  Yes
             * RPDO No
             * TPDO No
             * Network configuration initialized
             ***********************************************************************/

            /* wait for all slaves to reach PRE_OP state */
            printf("start PRE_OP request\n");
            ec_slave[0].state = EC_STATE_PRE_OP;
            ec_writestate(0);
            chk = 200;
            inc = 0;
            do
            {
                current_state = ec_statecheck(0, EC_STATE_PRE_OP, 3000 + inc * 15); inc++;
                printf(" state=0x%2.2x\n", current_state);
            } while (chk-- && (current_state != EC_STATE_PRE_OP));
            printf("current state is 0x%2.2x and 0x%2.2x was requested\n", current_state, EC_STATE_PRE_OP);

            /* check for IS620N */
            if (ec_slave[1].eep_id == 0x000c0108)
            {
                WRITE(1, 0x1c12, 0x00, buf16, 0, "Number of assigned RPDOs");
                WRITE(1, 0x1c12, 0x01, buf16, 0x1600, "1st PDO mapping object index of assigned RPDO");
                WRITE(1, 0x1600, 0x00, buf8, 0, "Number of mapped application objects in RPDO1");
                WRITE(1, 0x1600, 0x01, buf32, 0x60400010, "1st application object");
                WRITE(1, 0x1600, 0x02, buf32, 0x607a0020, "2nd application object");
                WRITE(1, 0x1600, 0x03, buf32, 0x60ff0020, "3rd application object");
                WRITE(1, 0x1600, 0x04, buf32, 0x60710010, "4th application object");
                WRITE(1, 0x1600, 0x00, buf8, 4, "Number of mapped application objects in RPDO1");
                WRITE(1, 0x1c12, 0x00, buf16, 1, "Number of assigned RPDOs");

                WRITE(1, 0x1c13, 0x00, buf16, 0, "Number of assigned TPDOs");
                WRITE(1, 0x1c13, 0x01, buf16, 0x1a00, "1st PDO mapping object index of assigned TPDO");
                WRITE(1, 0x1a00, 0x00, buf8, 0, "Number of mapped application objects in TPDO1");
                WRITE(1, 0x1a00, 0x01, buf32, 0x60410010, "1st application object");
                WRITE(1, 0x1a00, 0x02, buf32, 0x603f0010, "2nd application object");
                WRITE(1, 0x1a00, 0x03, buf32, 0x60640020, "3rd application object");
                WRITE(1, 0x1a00, 0x04, buf32, 0x606c0020, "4h application object");
                WRITE(1, 0x1a00, 0x05, buf32, 0x60770010, "5th application object");
                WRITE(1, 0x1a00, 0x06, buf32, 0x60610008, "6th application object");
                // WRITE(1, 0x1a00, 0x07, buf32, 0x203f0020, "dummy 7"); // de test factory error
                WRITE(1, 0x1a00, 0x00, buf8, 6, "Number of mapped application objects in TPDO1");
                WRITE(1, 0x1c13, 0x00, buf8, 1, "Number of assigned TPDOs");

                WRITE(1, 0x6060, 0x00, buf8, 8, "Modes of operation PV");

                READ(1, 0x1c12, 0x01, buf16, "1st PDO mapping object index of assigned RPDO");
                READ(1, 0x1600, 0x00, buf8, "Number of mapped application objects in RPDO1");
                READ(1, 0x1c13, 0x01, buf16, "1st PDO mapping object index of assigned TPDO");
                READ(1, 0x1a00, 0x00, buf8, "Number of mapped application objects in TPDO1");
                READ(1, 0x6060, 0x00, buf8, "Modes of operation");

                // ne trebuie pentru CSP
                READ(1, 0x60b0, 0x00, buf32, "Position offset");
                READ(1, 0x6065, 0x00, threshold_position_6065h, "Following error window");
                READ(1, 0x6067, 0x00, buf32, "Position window");
                READ(1, 0x6068, 0x00, buf32, "Position window time");
                READ(1, 0x6091, 0x00, buf8, "Gear ratio - Highest sub-index supported");
                READ(1, 0x6091, 0x01, buf32, "Gear ratio - Motor revolutions");
                READ(1, 0x6091, 0x02, buf32, "Gear ratio - Shaft revolutions");

            }

            /* configure SYNC0 */
            ec_dcsync0(1, TRUE, CYCLE_TIME, 0);
            printf("ec_dcsync0 ok\n");

            /* configure DC options for every DC capable slave found in the list */
            hasdc = ec_configdc();
            printf("ec_configdc %d\n", hasdc);

            /* configure PDO map */
            ec_config_map(&IOmap);

            pod_out1 = (out_IS620N_t *)(ec_slave[1].outputs);
            pod_in1 = (in_IS620N_t *)(ec_slave[1].inputs);

            // citim SYNC0/1 cycle size
            // wkc = ec_FPRD(ec_slave[1].configadr, ECT_REG_DCCYCLE0, 4, &buf32, 200);
            // printf("SYCN0 wkc=%d cycle=0x%8.8x\n", wkc, buf32);

            // asteptam 100.000.000 ns pana cand se activeaza SYNC0
            // aceasta valoare este in config-ul de SOEM
            osal_usleep(50 * CYCLE_TIME / 1000);

            /* read indevidual slave state and store in ec_slave[] */
            ec_readstate();

            for (cnt = 1; cnt <= ec_slavecount; cnt++)
            {
                printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d hasDC:%d\n",
                       cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                       ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
                printf("         Out:%p,%4d In:%p,%4d\n",
                       ec_slave[cnt].outputs, ec_slave[cnt].Obytes, ec_slave[cnt].inputs, ec_slave[cnt].Ibytes);
                /* check for IS620N or XXX */
                if (!digout && ((ec_slave[cnt].eep_id == 0x000c0108) || (ec_slave[cnt].eep_id == 0x11111111)))
                {
                    digout = ec_slave[cnt].outputs;
                }
            }
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;

            /*
            if (oloop > 8)
                oloop = 8;
            */

            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;

            /*
            if (iloop > 8)
                iloop = 8;
            */

            /*********************************************************************** 
             * SAFE_OP
             * SDO  Yes
             * RPDO No
             * TPDO Yes
             * Distributed clock mode used
             ***********************************************************************/

            /* wait for all slaves to reach SAFE_OP state */
            printf("start EC_STATE_SAFE_OP request\n");
            ec_slave[0].state = EC_STATE_SAFE_OP;
            ec_writestate(0);

            chk = 200;
            inc = 0;
            do
            {
                current_state = ec_statecheck(0, EC_STATE_SAFE_OP, 9000 + inc * 45); inc++;
                printf(" state=0x%2.2x\n", current_state);
            } while (chk-- && (current_state != EC_STATE_SAFE_OP));
            printf("current state is 0x%2.2x and 0x%2.2x was requested\n", current_state, EC_STATE_SAFE_OP);

            /*********************************************************************** 
             * OPERATIONAL
             * SDO  Yes
             * RPDO Yes
             * TPDO Yes
             * Normal operational state
             ***********************************************************************/

            /* wait for all slaves to reach OPERATIONAL state */
            printf("start EC_STATE_OPERATIONAL request\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            /* activate cyclic process data */
            dorun = 1;
            /* wait for a little to sync the master clock */
            osal_usleep(1000000);

            ec_writestate(0);

            chk = 200; 
            inc = 0;
            do
            {
                current_state = ec_statecheck(0, EC_STATE_OPERATIONAL, 9000 + inc * 45); inc++;
                printf(" state=0x%2.2x\n", current_state);
            } while (chk-- && (current_state != EC_STATE_OPERATIONAL));

            printf("current state is 0x%2.2x and 0x%2.2x was requested\n", current_state, EC_STATE_OPERATIONAL);

            printf("---> ec_DCtime=%ld\n", ec_DCtime);

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("OPERATIONAL state reached for all slaves.\n");
                inOP = TRUE;

                /* acyclic loop 5000 x 20ms = 10s */
                for (int i = 1; i <= 500000; i++)
                {
                    printf("PDO %5d, wck %1d, cycl %5.1f:%7.1f, shft %6.1f:%7.1f, eth %6ld, dpos %8ld, O:",
                           dorun, wkc, delta_cycle_avg, delta_cycle_std, delta_shift_avg, delta_shift_std,
                           ethercat_time, delta_position);
                    /*
                    for (int j = 0; j < oloop; j++)
                    {
                        printf(" %2.2x", *(ec_slave[0].outputs + j));
                    }
                    */
                    printf(" %4.4x %9d", pod_out1->control_word_6040, pod_out1->target_position_607a);

                    printf(" I:");
                    /*
                    for (int j = 0; j < iloop; j++)
                    {
                        printf(" %2.2x", *(ec_slave[0].inputs + j));
                    }
                    */
                    printf(" %4.4x %4.4x %9d %9d", pod_in1->status_word_6041, pod_in1->error_code_603f, pod_in1->position_actual_value_6064, pod_in1->velocity_actual_value_606c);

                    printf("\n");
                    fflush(stdout);
                    osal_usleep(1e6 * 2); // 2 sec
                }
                dorun = 0;
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached OPERATIONAL state.\n");
                ec_readstate();
                for (int i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("Request safe operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_SAFE_OP;
            /* request SAFE_OP state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End redundant test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec >= NSEC_PER_SEC)
    {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime) // reftime -> ec_DCtime, cycletime -> data cycle, offsettime -> toff
{
    static int64 integral = 0;
    int64 delta;
    /* set linux sync point SHIFT_TIME ns later than DC sync, just as example */
    delta = (reftime - SHIFT_TIME) % cycletime;
    if (delta > (cycletime / 2))
    {
        delta = delta - cycletime;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }
    *offsettime = -(delta * Kp) - (integral * Ki); // Kp = 0.1, Ki = 0.0005;
    delta_shift = delta;
    pid_integral = integral;

    /* calculam statistici pentru shift time *********************************/
    delta_shift_arr[delta_shift_crt] = delta_shift;

    // increment delta_shift_samples
    delta_shift_samples++;
    if (delta_shift_samples > SAMPLE_SIZE)
    {
        delta_shift_samples = SAMPLE_SIZE;
    }

    // calculate sum
    delta_shift_sum = 0;
    for (int i = 0; i < delta_shift_samples; i++)
    {
        delta_shift_sum += delta_shift_arr[i];
    }
    delta_shift_avg = delta_shift_sum / delta_shift_samples;

    // calculate std
    delta_shift_sum = 0;
    for (int i = 0; i < delta_shift_samples; i++)
    {
        delta_shift_sum += pow(delta_shift_arr[i] - delta_shift_avg, 2);
    }
    delta_shift_std = sqrt(delta_shift_sum / delta_shift_samples);

    // incrementam delta_shift_crt
    delta_shift_crt++;
    if (delta_shift_crt > SAMPLE_SIZE)
    {
        // avem valori de la 0 la SAMPLE_SIZE (deschis)
        delta_shift_crt = 0;
    }
}

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
    struct timespec ts, tleft;

    (void)ptr;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    toff = 0;
    dorun = 0;

    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        ec_send_processdata();
    }

    // l-am pus noi
    wkc = ec_receive_processdata(EC_TIMEOUTRX);

    while (1)
    {
        /* calulate toff to get linux time and DC synced */
        ec_sync(ec_DCtime, CYCLE_TIME, &toff);

        /* calculate next cycle start */
        add_timespec(&ts, CYCLE_TIME + toff);

        /* wait to cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

        /* 
         * cand folosim TIMER_ABSTIME, tleft nu se foloseste 
         * https://man7.org/linux/man-pages/man2/clock_nanosleep.2.html
         * tleft putem vedea valoarea ramasa pana la executie 
         */

        if (dorun > 0)
        {
            clock_gettime(CLOCK_MONOTONIC, &prev_time);
            prev_DCtime = ec_DCtime;

            // send + receive datagram
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRX);

            clock_gettime(CLOCK_MONOTONIC, &now_time);
            ethercat_time = (now_time.tv_sec - prev_time.tv_sec) * NSEC_PER_SEC + (now_time.tv_nsec - prev_time.tv_nsec);
            delta_cycle = CYCLE_TIME - (ec_DCtime - prev_DCtime);

            // calculate statistics for cycle
            delta_cycle_arr[delta_cycle_crt] = delta_cycle;

            // increment delta_cycle_samples
            delta_cycle_samples++;
            if (delta_cycle_samples > SAMPLE_SIZE)
            {
                delta_cycle_samples = SAMPLE_SIZE;
            }

            // calculate sum
            delta_cycle_sum = 0;
            for (int i = 0; i < delta_cycle_samples; i++)
            {
                delta_cycle_sum += delta_cycle_arr[i];
            }
            delta_cycle_avg = delta_cycle_sum / delta_cycle_samples;

            // calculate std
            delta_cycle_sum = 0;
            for (int i = 0; i < delta_cycle_samples; i++)
            {
                delta_cycle_sum += pow(delta_cycle_arr[i] - delta_cycle_avg, 2);
            }
            delta_cycle_std = sqrt(delta_cycle_sum / delta_cycle_samples);

            // incrementam delta_cycle_crt
            delta_cycle_crt++;
            if (delta_cycle_crt > SAMPLE_SIZE)
            {
                // avem valori de la 0 la SAMPLE_SIZE (deschis)
                delta_cycle_crt = 0;
            }

            /*
            * Drive state machine transistions -> move to Operational
            *   0 -> 6 -> 7 -> 15
            *   5.3.3 State Machine
            *   Control command and state switchover -> pag. 180 (181/633) Advanced User Guide
            *   https://en.nanotec.com/products/manual/PD2C_CAN_EN/general%252Fds402_power_state_machine.html?cHash=21cd60eb6c108ef18a93811237a392c5
            *   5.3.7 Indication
            *   _88xx
            *   prima indica Communication status -> 1 (Init), 2 (Pre-operational), 4 (Safe-operational), 8 (Operational)
            *   a doua indica Control mode -> 1 (Profile position), 3 (Profile velocity), 6 (Homing mode), 8 (Cyclic synchronous position), 9 (Cyclic synchronous velocity)
            *   ultimele doua indica Servo status -> nr (not ready), ry (ready), rn (run)
            */

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                
                /* statusword                                       State                  mask  value  transition command
                 * 0x0018 0000.0000.0001.1000  xxxx.xxxx.x0xx.1000  Fault                  0x4f  0x08   13=0x80    Fault reset
                 */
                if ((pod_in1->status_word_6041 & 0x4f) == 0x08)
                {
                    pod_out1->control_word_6040 = 0x80;
                }
                
                /* statusword                                       State                  mask  value  transition command
                 * 0x0250 0000.0010.0101.0000  xxxx.xxxx.x1xx.0000  Switch on disabled     0x4f  0x40   1=0x06     Shutdown
                 */
                if ((pod_in1->status_word_6041 & 0x4f) == 0x40)
                {
                    pod_out1->control_word_6040 = 0x06;
                }

                /* statusword                                       State                  mask  value  transition command
                 * 0x0231 0000.0010.0011.0001  xxxx.xxxx.x01x.0001  Ready to switch on     0x6f  0x21   2=0x07     Switch on   
                 */
                if ((pod_in1->status_word_6041 & 0x6f) == 0x21)
                {
                    pod_out1->control_word_6040 = 0x07;
                }

                /* statusword                                       State                  mask  value  transition command
                 * 0x0233 0000.0010.0011.0011  xxxx.xxxx.x01x.0011  Switched on            0x6f  0x23   3=0x0f     Enable operation
                 */
                if ((pod_in1->status_word_6041 & 0x6f) == 0x23)
                {
                    pod_out1->control_word_6040 = 0x0f;
                }

                /* statusword                                       State                  mask  value  transition command
                 * 0x1637 0001.0110.0011.0111  xxxx.xxxx.x01x.0111  Operation enabled      0x6f  0x27   3=0x0f
                 */
                if ((pod_in1->status_word_6041 & 0x6f) == 0x27)
                {
                    // suntem in Operation enabled si putem face operatiuni
                    pod_out1->control_word_6040 = 0x1f;

                    /*
                     * pag 188 din Advanced User Guide IS620N
                     *
                     * 60B0h = Position offset -> default = 0
                     *   Target positon = 607Ah + 60B0h (607Ah este chiar Target position)
                     * 
                     * 6067h Position window -> default = 734
                     * 6068h Position window time -> default = 0
                     *   cand am atins +/-6067h si timpul a atins 6068h, servo drive considera ca am atins pozitia 
                     *     si face 6041h bit10 = 1
                     * 
                     * 6065h Following error window -> default = 3435868
                     *   da Er.B00 daca depasim +/-6065h
                     * 
                     */

                    
                    // pod_out1->target_velocity_60ff = 25000;  // 24780     ->   300 mm/s
                    // 958128942 -> 96000 mm

                    delta_position = pod_out1->target_position_607a - pod_in1->position_actual_value_6064;

                    // trebuie sa urcam cu pozitia
                    if (pod_in1->position_actual_value_6064 > 1e8)
                    {
                        direction = -1; // incepem sa ne ducem in jos
                    }
                    if (pod_in1->position_actual_value_6064 < -1e8)
                    {
                        direction = 1; // incepem sa ne ducem in sus
                    }

                    pod_out1->target_position_607a = pod_in1->position_actual_value_6064 + direction * (int64)(threshold_position_6065h / 8.26);

                }
            }

            dorun++;
        }
    }
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
    int slave;
    volatile int wkc_now;

    (void)ptr;

    while (1)
    {
        if (inOP && (((wkc < expectedWKC) && (wkc >= 0)) || ec_group[currentgroup].docheckstate))
        {
            wkc_now = wkc;

            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }

                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL -> wkc %d\n", wkc_now);
        }
        osal_usleep(10000);
    }
}

#define stack64k (64 * 1024)

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    int cycle_time_us = CYCLE_TIME / 1000; // us

    printf("SOEM (Simple Open EtherCAT Master)\nREDundancy test\n");

    dorun = 0;

    /* create RT thread */
    osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void *)&cycle_time_us);

    /* create thread to handle slave error handling in OP */
    osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);

    /* start acyclic part */
    redtest();

    printf("End program\n");

    return (0);
}