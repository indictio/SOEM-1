/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : red_test [ifname1] [ifname2] [cycletime]
 * ifname is NIC interface, f.e. eth0
 * cycletime in us, f.e. 500
 *
 * This is a redundancy test.
 *
 * (c)Arthur Ketels 2008
 *
 * https://github.com/OpenEtherCATsociety/SOEM/issues/520
 *
 * 1. Call ec_config_init() to move from INIT to PRE-OP state
 * 2. Do slave-specific configurations via SDO communication
 * 3. Set ecx_context.manualstatechange = 1. Map PDOs for all slaves by calling ec_config_map()
 * 5. Manually call the transition to SAFE-OP (some slaves require starting sync0 here, but this is a violation of the EtherCAT protocol spec so it is not the default)
 * 4. Call ec_configdc() in PRE-OP state
 * 6. Do 10,000 process data cycles in SAFE-OP state
 * 6b) should then be to start the regular PDO transfer at the proper interval
 * 7. Synchronise the slave and master clock by setting the master clock = slave reference clock
 * 8. Call ec_dcsync0() for the reference clock slave
 * 9. Wait for a few seconds
 * 10. Transition to OP state
 *
 * din ESI-ul lui Inovance
 * PreopTimeout=3000
 * SafeopOpTimeout=9000 (standard 10000)
 * BackToInitTimeout=5000
 * BackToSafeopTimeout=200
 * Mailbox
 * RequestTimeout=100
 * ResponseTimeout=2000
 *
 * in standard avem INIT>PRE-OPERATIONAL>SAFE-OPERATIONAL>OPERATIONAL
 * in ESI avem timeout doar de PRE-OP, SAFE-OP
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
#define EC_TIMEOUTOP 1000000 // us
#define CYCLE_TIME 1000000   // ns
#define EC_TIMEOUTMON 500

#define MODE_PP 1
#define MODE_HP 6
#define MODE_CSP 8
#define RUNNING_MODE CSP_MODE

int mode_of_operation = MODE_CSP;
struct sched_param schedp;
char IOmap[4096];
pthread_t thread1, thread2;
struct timeval tv, t1, t2;
int dorun = 0;
int deltat, tmax = 0;
int64 toff, gl_delta;
int DCdiff;
int os;
uint8 ob;
uint16 ob2;
uint8 *digout = 0;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
int64 threshold_position_6065h;

in_IS620N_t *pod_in_1;
out_IS620N_t *pod_out_1;
in_IS620N_t *pod_in_2;
out_IS620N_t *pod_out_2;
in_IS620N_t *pod_in_3;
out_IS620N_t *pod_out_3;
in_IS620N_t *pod_in_4;
out_IS620N_t *pod_out_4;
in_IS620N_t *pod_in_5;
out_IS620N_t *pod_out_5;
in_IS620N_t *pod_in_6;
out_IS620N_t *pod_out_6;

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

/* 6.4.4 Recommended Configuration (CSP pag. 191)
 * RPDO
 * 6040 Control word
 * 607A Target position
 * TPDO
 * 6041 Status work
 * 6064 Position actual value
 */

int IS620N_setup(uint16 slave)
{
    int retval;
    uint32 buf32;
    uint16 buf16;
    uint8 buf8;

    retval = 0;

    // setam modul de operare inainte de a seta PDO
    WRITE(slave, 0x6060, 0x00, buf8, 8, "Modes of operation CSP");

    // partea comuna a RPDO
    WRITE(slave, 0x1c12, 0x00, buf16, 0, "Number of assigned RPDOs");
    WRITE(slave, 0x1c12, 0x01, buf16, 0x1600, "1st PDO mapping object index of assigned RPDO");
    WRITE(slave, 0x1600, 0x00, buf8, 0, "Number of mapped application objects in RPDO1");
    WRITE(slave, 0x1600, 0x01, buf32, 0x60400010, "1st application object"); // Control word
    WRITE(slave, 0x1600, 0x02, buf32, 0x60600008, "2nd application object"); // Modes of operation

    if (mode_of_operation == MODE_CSP)
    {
        WRITE(slave, 0x1600, 0x03, buf32, 0x607A0020, "3rd application object"); // Target position
        WRITE(slave, 0x1600, 0x00, buf8, 3, "Number of mapped application objects in RPDO1");
    }
    else if (mode_of_operation == MODE_HP)
    {
        WRITE(slave, 0x1600, 0x03, buf32, 0x60980008, "3rd application object");
        WRITE(slave, 0x1600, 0x04, buf32, 0x60990120, "4th application object");
        WRITE(slave, 0x1600, 0x05, buf32, 0x60990220, "5th application object");
        WRITE(slave, 0x1600, 0x06, buf32, 0x609A0020, "6th application object");
        WRITE(slave, 0x1600, 0x00, buf8, 6, "Number of mapped application objects in RPDO1");
    }
    WRITE(slave, 0x1c12, 0x00, buf16, 1, "Number of assigned RPDOs");

    WRITE(slave, 0x1c13, 0x00, buf16, 0, "Number of assigned TPDOs");
    WRITE(slave, 0x1c13, 0x01, buf16, 0x1a00, "1st PDO mapping object index of assigned TPDO");
    WRITE(slave, 0x1a00, 0x00, buf8, 0, "Number of mapped application objects in TPDO1");
    WRITE(slave, 0x1a00, 0x01, buf32, 0x60410010, "1st application object"); // Status word
    WRITE(slave, 0x1a00, 0x02, buf32, 0x60610008, "2nd application object"); // Modes of operation display
    WRITE(slave, 0x1a00, 0x03, buf32, 0x603f0010, "3rd application object"); // Error code
    WRITE(slave, 0x1a00, 0x04, buf32, 0x60640020, "4th application object"); // Position actual value
    WRITE(slave, 0x1a00, 0x05, buf32, 0x606c0020, "5th application object"); // Velocity actual value
    WRITE(slave, 0x1a00, 0x06, buf32, 0x60770010, "6th application object"); // Torque actual value

    WRITE(slave, 0x1a00, 0x00, buf8, 6, "Number of mapped application objects in TPDO1");
    WRITE(slave, 0x1c13, 0x00, buf8, 1, "Number of assigned TPDOs");

    READ(slave, 0x1c12, 0x01, buf16, "1st PDO mapping object index of assigned RPDO");
    READ(slave, 0x1600, 0x00, buf8, "Number of mapped application objects in RPDO1");
    READ(slave, 0x1c13, 0x01, buf16, "1st PDO mapping object index of assigned TPDO");
    READ(slave, 0x1a00, 0x00, buf8, "Number of mapped application objects in TPDO1");
    READ(slave, 0x6060, 0x00, buf8, "Modes of operation");

    // set Gear box
    WRITE(slave, 0x6091, 0x01, buf32, 1048575, "Gear ratio - Motor revolutions");
    WRITE(slave, 0x6091, 0x02, buf32, 10000, "Gear ratio - Shaft revolutions");

    if (mode_of_operation == MODE_CSP)
    {
        READ(slave, 0x60B0, 0x00, buf32, "Position offset");
        READ(slave, 0x6065, 0x00, threshold_position_6065h, "Following error window");
        READ(slave, 0x6067, 0x00, buf32, "Position window");
        READ(slave, 0x6068, 0x00, buf32, "Position window time");
        READ(slave, 0x6091, 0x00, buf8, "Gear ratio - Highest sub-index supported");
        READ(slave, 0x6091, 0x01, buf32, "Gear ratio - Motor revolutions");
        READ(slave, 0x6091, 0x02, buf32, "Gear ratio - Shaft revolutions");
    }

    while (EcatError)
        printf("EcatErro %s", ec_elist2string());

    printf("IS620N slave %d set, retval = %d\n", slave, retval);
    return 1;
}

void redtest(char *ifname, char *ifname2)
{
    int i, chk;
    uint16 current_state;

    printf("Starting Redundant test\n");

    // initialise SOEM, bind socket to ifname
    (void)ifname2;
    //   if (ec_init_redundant(ifname, ifname2))
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);

        // manual state change
        ecx_context.manualstatechange = 1;

        // find and auto-config slaves
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            // suntem in init state?
            ec_readstate();
            for (int slv = 1; slv <= ec_slavecount; slv++)
            {
                printf("Slave %d State=0x%2.2x (should be 0x01) StatusCode=0x%4.4x : %s\n",
                       slv,
                       ec_slave[slv].state,
                       ec_slave[slv].ALstatuscode,
                       ec_ALstatuscode2string(ec_slave[slv].ALstatuscode));
            }

            printf("Request INIT state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            ec_writestate(0);
            chk = 10;
            do
            {
                current_state = ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTOP);
            } while (chk-- && (current_state != EC_STATE_INIT));
            printf("current state is 0x%2.2x vs 0x%2.2x in %d cycles\n", current_state, EC_STATE_INIT, 10 - chk);

            // unconfigure SYNC0
            printf("ec_dcsync0 unconfigure\n");
            for (int slv = 1; slv <= ec_slavecount; slv++)
            {
                ec_dcsync0(slv, FALSE, 0, 0);
            }

            printf("Request PRE-OPERATIONAL state for all slaves\n");
            ec_slave[0].state = EC_STATE_PRE_OP;
            ec_writestate(0);
            chk = 10;
            do
            {
                current_state = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTOP);
            } while (chk-- && (current_state != EC_STATE_PRE_OP));
            printf("current state is 0x%2.2x vs 0x%2.2x in %d cycles\n", current_state, EC_STATE_PRE_OP, 10 - chk);

            // setam servo drivers
            for (int slv = 1; slv <= ec_slavecount; slv++)
            {
                if ((ec_slave[slv].eep_man == 0x00100000) && (ec_slave[slv].eep_id == 0x000c0108))
                {
                    printf("Found %s at position %d\n", ec_slave[slv].name, slv);
                    // link slave specific setup to preop->safeop hook
                    // ec_slave[slv].PO2SOconfig = IS620N_setup;
                    IS620N_setup(slv);
                }
            }

            // map slave's PDO into IOmap.inputs and IOmap.outputs
            ec_config_map(&IOmap);
            pod_out_1 = (out_IS620N_t *)(ec_slave[1].outputs);
            pod_in_1 = (in_IS620N_t *)(ec_slave[1].inputs);
            pod_out_2 = (out_IS620N_t *)(ec_slave[2].outputs);
            pod_in_2 = (in_IS620N_t *)(ec_slave[2].inputs);
            pod_out_3 = (out_IS620N_t *)(ec_slave[3].outputs);
            pod_in_3 = (in_IS620N_t *)(ec_slave[3].inputs);
            pod_out_4 = (out_IS620N_t *)(ec_slave[4].outputs);
            pod_in_4 = (in_IS620N_t *)(ec_slave[4].inputs);
            pod_out_5 = (out_IS620N_t *)(ec_slave[5].outputs);
            pod_in_5 = (in_IS620N_t *)(ec_slave[5].inputs);
            pod_out_6 = (out_IS620N_t *)(ec_slave[6].outputs);
            pod_in_6 = (in_IS620N_t *)(ec_slave[6].inputs);

            // configure DC options for every DC capable slave found in the list
            printf("ec_configdc\n");
            ec_configdc();

            // static drift compensation ~15.000 times - register 0x0910
            for (int i = 0; i < 15000; i++)
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
            }

            // activate cyclic process data
            // conform EtherCAT Protocol - aici se porneste
            // asta va porni OSAL_THREAD_FUNC_RT ecatthread
            dorun = 1;

            chk = 3;
            do
            {
                osal_usleep(1e6);
                printf(" %6ld\n", gl_delta);
            } while (chk-- || (llabs(gl_delta) == 0));
            printf("delta %6ld after %d cycles\n", gl_delta, 10 - chk);

            printf("ec_dcsync0\n");
            for (int slv = 1; slv <= ec_slavecount; slv++)
            {
                ec_dcsync0(slv, TRUE, CYCLE_TIME, 0);
            }

            chk = 3;
            do
            {
                osal_usleep(1e6);
                printf(" %6ld\n", gl_delta);
            } while (chk-- || (llabs(gl_delta) == 0));
            printf("delta %6ld after %d cycles\n", gl_delta, 10 - chk);

            // read indevidual slave state and store in ec_slave[]
            ec_readstate();

            printf("Request SAFE-OPERATIONAL state for all slaves\n");
            ec_slave[0].state = EC_STATE_SAFE_OP;
            ec_writestate(0);
            chk = 10;
            do
            {
                current_state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTOP);
            } while (chk-- && (current_state != EC_STATE_SAFE_OP));
            printf("current state is 0x%2.2x vs 0x%2.2x in %d cycles\n", current_state, EC_STATE_SAFE_OP, 10 - chk);

            // read indevidual slave state and store in ec_slave[]
            ec_readstate();
            for (int cnt = 1; cnt <= ec_slavecount; cnt++)
            {
                printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                       cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                       ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
                printf("         Out:%p,%4d In:%p,%4d\n",
                       ec_slave[cnt].outputs, ec_slave[cnt].Obytes, ec_slave[cnt].inputs, ec_slave[cnt].Ibytes);
                /* check for EL2004 or EL2008 */
                if (!digout && ((ec_slave[cnt].eep_id == 0x0af83052) || (ec_slave[cnt].eep_id == 0x07d83052)))
                {
                    digout = ec_slave[cnt].outputs;
                }
            }
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            for (int i = 0; i < 10; i++)
            {
                pod_out_1->modes_of_operation_6060 = mode_of_operation;
                pod_out_1->control_word_6040 = 0x0250;
                pod_out_1->target_position_607a = pod_in_1->position_actual_value_6064;
                pod_out_2->modes_of_operation_6060 = mode_of_operation;
                pod_out_2->control_word_6040 = 0x0250;
                pod_out_2->target_position_607a = pod_in_2->position_actual_value_6064;
                pod_out_3->modes_of_operation_6060 = mode_of_operation;
                pod_out_3->control_word_6040 = 0x0250;
                pod_out_3->target_position_607a = pod_in_3->position_actual_value_6064;
                pod_out_4->modes_of_operation_6060 = mode_of_operation;
                pod_out_4->control_word_6040 = 0x0250;
                pod_out_4->target_position_607a = pod_in_4->position_actual_value_6064;
                pod_out_5->modes_of_operation_6060 = mode_of_operation;
                pod_out_5->control_word_6040 = 0x0250;
                pod_out_5->target_position_607a = pod_in_5->position_actual_value_6064;
                pod_out_6->modes_of_operation_6060 = mode_of_operation;
                pod_out_6->control_word_6040 = 0x0250;
                pod_out_6->target_position_607a = pod_in_6->position_actual_value_6064;

                osal_usleep(CYCLE_TIME / 1000);

                printf("I: %4.4x %4.4x %9d %9d\n",
                       pod_in_1->status_word_6041,
                       pod_in_1->error_code_603f,
                       pod_in_1->position_actual_value_6064,
                       pod_in_1->velocity_actual_value_606c);
            }

            printf("Request OPERATIONAL state for all slaves\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            // send one valid process data to make outputs in slaves happy
            // ec_send_processdata();
            // ec_receive_processdata(EC_TIMEOUTRET);
            // writestate
            ec_writestate(0);
            chk = 10;
            do
            {
                current_state = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTOP);
            } while (chk-- && (current_state != EC_STATE_OPERATIONAL));
            printf("current state is 0x%2.2x vs 0x%2.2x in %d cycles\n", current_state, EC_STATE_OPERATIONAL, 10 - chk);

            // oloop = ec_slave[0].Obytes;
            // iloop = ec_slave[0].Ibytes;

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves.\n");

                // asta va porni OSAL_THREAD_FUNC ecatcheck
                inOP = TRUE;

                pod_out_1->modes_of_operation_6060 = mode_of_operation;
                pod_out_1->control_word_6040 = 0x0250;
                pod_out_1->target_position_607a = pod_in_1->position_actual_value_6064;
                pod_out_2->modes_of_operation_6060 = mode_of_operation;
                pod_out_2->control_word_6040 = 0x0250;
                pod_out_2->target_position_607a = pod_in_2->position_actual_value_6064;
                pod_out_3->modes_of_operation_6060 = mode_of_operation;
                pod_out_3->control_word_6040 = 0x0250;
                pod_out_3->target_position_607a = pod_in_3->position_actual_value_6064;
                pod_out_4->modes_of_operation_6060 = mode_of_operation;
                pod_out_4->control_word_6040 = 0x0250;
                pod_out_4->target_position_607a = pod_in_4->position_actual_value_6064;
                pod_out_5->modes_of_operation_6060 = mode_of_operation;
                pod_out_5->control_word_6040 = 0x0250;
                pod_out_5->target_position_607a = pod_in_5->position_actual_value_6064;
                pod_out_6->modes_of_operation_6060 = mode_of_operation;
                pod_out_6->control_word_6040 = 0x0250;
                pod_out_6->target_position_607a = pod_in_6->position_actual_value_6064;

                /* acyclic loop 5000 x 20ms = 10s */
                for (i = 1; i <= 5000; i++)
                {
                    printf("Pdc %5d , Wck %3d, DCtime %6ld, dt %6ld, ",
                           dorun,
                           wkc,
                           ec_DCtime,
                           gl_delta);

                    /*
                    for (j = 0; j < oloop; j++)
                    {
                        printf(" %2.2x", *(ec_slave[0].outputs + j));
                    }
                    printf(" I:");
                    for (j = 0; j < iloop; j++)
                    {
                        printf(" %2.2x", *(ec_slave[0].inputs + j));
                    }
                    */
                    printf("O: %4.4x %9d ",
                           pod_out_1->control_word_6040,
                           pod_out_1->target_position_607a);
                    printf("I: %4.4x %4.4x %9d %9d",
                           pod_in_1->status_word_6041,
                           pod_in_1->error_code_603f,
                           pod_in_1->position_actual_value_6064,
                           pod_in_1->velocity_actual_value_606c);

                    printf("\n");
                    fflush(stdout);
                    osal_usleep(2 * 1e6);
                }
                dorun = 0;
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }

            printf("Request SAFE-OPERATIONAL state for all slaves\n");
            ec_slave[0].state = EC_STATE_SAFE_OP;
            ec_writestate(0);
            // wait for all slaves to reach SAFE-OPERATIONAL state
            current_state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTOP);
            printf("current state is 0x%2.2x vs 0x%2.2x\n", current_state, EC_STATE_SAFE_OP);
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
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;

    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
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
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
}

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    // Convert from timeval to timespec
    ht = (ts.tv_nsec / 1000000) + 1; // round to nearest ms
    ts.tv_nsec = ht * 1000000;
    if (ts.tv_nsec >= NSEC_PER_SEC)
    {
        ts.tv_sec++;
        ts.tv_nsec -= NSEC_PER_SEC;
    }
    cycletime = *(int *)ptr * 1000; // cycletime in ns
    toff = 0;
    dorun = 0;
    ec_send_processdata();

    while (1)
    {
        // calculate next cycle start
        add_timespec(&ts, cycletime + toff);
        // wait to cycle start
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
        if (dorun > 0)
        {
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            dorun++;
            // if we have some digital output, cycle
            if (digout)
                *digout = (uint8)((dorun / 16) & 0xff);

            ec_send_processdata();
        }
        // calulate toff to get linux time and DC synced
        ec_sync(ec_DCtime, cycletime, &toff);
    }
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
    int slave;

    (void)ptr;

    while (1)
    {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
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
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

#define stack64k (64 * 1024)

int main(int argc, char *argv[])
{
    int ctime;

    printf("SOEM (Simple Open EtherCAT Master)\nRedundancy test\n");

    if (argc > 3)
    {
        dorun = 0;
        ctime = atoi(argv[3]);

        /* create RT thread */
        osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void *)&ctime);

        /* create thread to handle slave error handling in OP */
        osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);

        /* start acyclic part */
        redtest(argv[1], argv[2]);
    }
    else
    {
        printf("Usage: red_test ifname1 ifname2 cycletime\nifname = eth0 for example\ncycletime in us\n");
    }

    printf("End program\n");

    return (0);
}