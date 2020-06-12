/****************************************************************************
 *
 *   Copyright 2019 NXP.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hello_example.cpp
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */

#include "nxpcup_race.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

// Image size (x,y) -> (79,52) pixels
const int x_left_origin = 19;
const int x_right_origin = 60;
const int y_origin = 52;

const double vmax = 1.0;

int count = 0;
const int count_stop = (int)(50 / vmax);

int8_t res;

roverControl control_saved;

roverControl raceTrack(Pixy2 &pixy)
{
	roverControl control{};

	float x0, x1, y0, y1, xe, ye;
	float sign, u, x_origin, norme, delta;

	res = pixy.line.getMainFeatures();

	pixy.line.vectors->print();

	// PX4_INFO("Pixy:\t%u vectors found", pixy.line.numVectors);

	// for (int i = 0; i <= pixy.line.numVectors; i++) {
	// 	pixy.line.vectors[i].print();
	// }

	// pixy.line.vectors[0].m_x0;
	// pixy.line.vectors[0].m_x1;
	// pixy.line.vectors[0].m_y0;
	// pixy.line.vectors[0].m_y1;

	// Control information:
	// speed: -1 (back) to +1 (front)
	// speed: (m/s) current
	// steering: -1 (left) to +1 (right)
	// steering: rad -pi (left) to +pi (right) current

	if (res & LINE_VECTOR) {
		x0 = (float)pixy.line.vectors->m_x0;
		x1 = (float)pixy.line.vectors->m_x1;
		y0 = (float)(52 - pixy.line.vectors->m_y0);
		y1 = (float)(52 - pixy.line.vectors->m_y1);

		sign = (y1 > y0) ? 1.0F : -1.0F;
		u = sign * (x1 - x0) / (y1 - y0);

		ye = y_origin;
		xe = x0 + u * (ye - y0);

		PX4_INFO("u: %f", (double)u);
		PX4_INFO("x1: %f", (double)x1);

		x_origin = (pixy.line.vectors->m_x0 < 40) ? x_left_origin : x_right_origin;

		norme = sqrt(pow(xe - x_origin, 2) + pow(ye, 2));

		delta = acos(ye / norme);
		delta = copysign(delta, xe - x_origin);

		control.speed = vmax * cos(0.75F * delta);
		control.steer = delta;

		control_saved = control;

		if (count != 0) { count = 0; }

		PX4_INFO("speed: %f [m/s]", (double)control.speed);
		PX4_INFO("delta: %f [deg]", (double)delta * 180.0 / M_PI);

	} else {
		if (count < count_stop) {
			control = control_saved;
			count++;

		} else {
			control.speed = 0.0;
			control.steer = 0.0;
		}
	}

	return control;
}
