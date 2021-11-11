#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

typedef struct PACKED
{
    uint16 control_word_6040;
    int8 modes_of_operation_6060;
    int32 target_position_607a;
} out_IS620N_t;

typedef struct PACKED
{
    uint16 status_word_6041;
    int8 modes_of_operation_display_6061;
    uint16 error_code_603f;
    int32 position_actual_value_6064;
    int32 velocity_actual_value_606c;
    int16 torque_actual_value_6077;
} in_IS620N_t;

typedef struct
{
    out_IS620N_t out_data;
    in_IS620N_t in_data;
} slave_IS620N_t;

typedef struct
{
    out_IS620N_t *ptout_data;
    in_IS620N_t *ptin_data;
} slave_IS620N_pt;