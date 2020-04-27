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

int x_origin = 39;
int y_origin = 52;
int count = 0;
int count_stop = 100;

int8_t res;

roverControl control_saved;

roverControl raceTrack(Pixy2 &pixy)
{
	roverControl control{};

	float delta, norme;

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
	// speed: (m/s)
	// steering: -1 (left) to +1 (right)
	// steering: rad -pi (left) to +pi (right)

	if (res & LINE_VECTOR) {
		norme = sqrt(pow(pixy.line.vectors->m_x1 - x_origin, 2) + pow(pixy.line.vectors->m_y1 - y_origin, 2));

		delta = acos(abs(pixy.line.vectors->m_y1 - y_origin) / norme);
		delta = copysign(delta, pixy.line.vectors->m_x1 - x_origin);

		control.speed = 0.25 * cos(0.5F * delta);
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
