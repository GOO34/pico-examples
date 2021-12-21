/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// #include <stdio.h>
// #include "pico/stdlib.h"

// int main() {
//     stdio_init_all();
//     while (true) {
//         printf("Hello, world!\n");
//         sleep_ms(1000);
//     }
//     return 0;
// }


#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#define FLAG_VALUE 123

int command;
int count;
int _pulse_per_round = 3000;
double _mm_per_pulse = 0.01;

void absolute_move(double current_position, double target_position, double accelerate_time_ms, double decelerate_time_ms, double max_speed, double initial_speed, double final_speed)
{
	int pulse_per_round = _pulse_per_round;
	double mm_per_pulse = _mm_per_pulse;
	/*
	* S = (Vi +Vmax) * Tacc / 2 + Vmax * (Tt - Tacc - Tde) + (Vmax + Vf) * Tde / 2;
	* Tmax = Tt - Tacc - Tde = (S - (1/2) * ((Vi +Vmax) * Tacc + (Vmax + Vf) * Tde))/ Vmax ;
	* Tt = Tmax + Tacc +Tde;
	*/

	int max_time_ms = (int)round(((target_position - current_position) - (1 / 2) * ((initial_speed + max_speed) * accelerate_time_ms + (max_speed + final_speed) * decelerate_time_ms)) / max_speed);
	//int total_time_ms = (int)round(max_time_ms - accelerate_time_ms - decelerate_time_ms);

	//最高速度-pulse週期(ms)
	int max_ms_per_pulse = mm_per_pulse / max_speed;
	if (max_ms_per_pulse < 1)
		max_ms_per_pulse = 1;
	//初始速度-pulse週期(ms)
	int inital_ms_per_pulse = mm_per_pulse / initial_speed;
	if (inital_ms_per_pulse < 1)
		inital_ms_per_pulse = 1;
	//最終速度-pulse週期(ms)
	int final_ms_per_pulse = mm_per_pulse / final_speed;
	if (final_ms_per_pulse < 1)
		final_ms_per_pulse = 1;

	//最高速度段-總共需要的pulse
	int max_pulse = max_speed * max_time_ms / mm_per_pulse;
	//加速度段-總共需要的pulse
	int accelerate_pulse = max_speed * max_time_ms / mm_per_pulse;
	//減速度段-總共需要的pulse
	int decelerate_pulse = max_speed * max_time_ms / mm_per_pulse;

	int accelerate_decrease_interval = (max_ms_per_pulse - inital_ms_per_pulse) / accelerate_pulse;
	int decelerate_increase_interval = (final_ms_per_pulse - max_ms_per_pulse) / accelerate_pulse;

	//加速段
	for (size_t i = 0; i < accelerate_pulse; i++)
	{
		int interval = inital_ms_per_pulse + i * accelerate_decrease_interval;
		rotate_pulse(interval);
	}
	//最高速段
	for (size_t i = 0; i < max_pulse; i++)
	{
		int interval = max_ms_per_pulse;
		rotate_pulse(interval);
	}
	//減速段
	for (size_t i = 0; i < decelerate_pulse; i++)
	{
		int interval = max_ms_per_pulse + i * decelerate_increase_interval;
		rotate_pulse(interval);
	}

}

void core1_entry() {

    multicore_fifo_push_blocking(FLAG_VALUE);

    uint32_t g = multicore_fifo_pop_blocking();

    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 1!\n");
    else
        printf("Its all gone well on core 1!");

    while (1)
        tight_loop_contents();
}

int main() {
    stdio_init_all();

	printf("Hello, multicore!\n");

    /// \tag::setup_multicore[]

    multicore_launch_core1(core1_entry);

    // Wait for it to start up

    uint32_t g = multicore_fifo_pop_blocking();

    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 0!\n");
    else {
        multicore_fifo_push_blocking(FLAG_VALUE);
        printf("It's all gone well on core 0!");
    }


    while (true) {


      	while (command > 48 && count > 1000000)
		{




		}


        
    }
    return 0;
}