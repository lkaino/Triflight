/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#define FILTER_TAPS 13

typedef struct filterStatePt1_s {
	float state;
	float RC;
	float constdT;
} filterStatePt1_t;

float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dt);
int8_t * filterGetFIRCoefficientsTable(uint8_t filter_level, uint32_t targetLooptime);
void filterApplyFIR(int16_t data[3], int16_t state[3][FILTER_TAPS], int8_t coeff[FILTER_TAPS]);
