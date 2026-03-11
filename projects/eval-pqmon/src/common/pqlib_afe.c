/*******************************************************************************
 *   @file   pqlib_afe.c
 *   @brief  Analog frontend data polling process
 *   @author Robert Budai (robert.budai@analog.com)
 ********************************************************************************
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "pqlib_afe.h"
#include "iio_pqm.h"

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @details Get AFE input.
 */
int get_afe_input()
{
	int status = 0;
	ADI_PQLIB_WAVEFORM *pWaveform = &(pqlibExample.inputWaveform);
	ADI_PQLIB_CYCLE_INPUT *pOneCycle = &(pqlibExample.inputCycle);
	ADI_PQLIB_1012_CYCLE_INPUT *p1012Cycle = &(pqlibExample.input1012Cycles);

	status = afe_read_status0((uint32_t *)&pOneCycle->STATUS0);
	if ((status == 0) && (pOneCycle->STATUS0 & BITM_STATUS0_RMSONERDY)) {
		status = afe_read_rms_one((uint32_t *)&pOneCycle->AVRMSONE, 3);
		if (status == 0) {
			status =
				afe_read_angle((uint16_t *)&pOneCycle->ANGL_VA_VB, 3, ANGLE_VOLTAGE);
		}
		if (status == 0) {
			status =
				afe_read_angle((uint16_t *)&pOneCycle->ANGL_IA_IB, 3, ANGLE_CURRENT);
		}
		if (status == 0) {
			status = afe_read_period((uint32_t *)&pOneCycle->PERIOD, 1);
		}
		if (status == 0) {
			status = afe_read_status_1((uint32_t *)&pOneCycle->STATUS1);
		}

		if (status == 0) {
			pOneCycle->isDataProcessed = 0;
			pOneCycle->sequenceNumber++;
		}
	}

	if ((status == 0) && (pOneCycle->STATUS0 & BITM_STATUS0_EGYRDY)) {
		status = read_all_energy(&pqlibExample.allEnergy);
	}
	
	if ((status == 0) && (pOneCycle->STATUS0 & BITM_STATUS0_COH_PAGE_RDY)) {
		status = afe_read_waveform(
				 (uint16_t *) & (pqlibExample.inputWaveform.waveform),
				 ADI_PQLIB_WAVEFORM_BLOCK_SIZE * ADI_PQLIB_TOTAL_WAVEFORM_CHANNELS);
		pWaveform->isDataProcessed = 0;
		pWaveform->sequenceNumber++;
		if (pqlibExample.no_os_cb_desc && !processData) {
			no_os_cb_write(pqlibExample.no_os_cb_desc,
				       (uint8_t *) & (pqlibExample.inputWaveform.waveform),
				       ADI_PQLIB_WAVEFORM_BLOCK_SIZE
				       * ADI_PQLIB_TOTAL_WAVEFORM_CHANNELS
				       * sizeof(uint16_t));
		}
	}

	if ((status == 0) && (pOneCycle->STATUS0 & BITM_STATUS0_RMS1012RDY)) {
		status = afe_read_rms_1012((uint32_t *)&p1012Cycle->AIRMS1012, 7);
		if (status == 0) {
			p1012Cycle->isDataProcessed = 0;
			p1012Cycle->sequenceNumber++;
		}
	}

	return status;
}

/**
 * @brief Read and process one energy type (HI+LO) from registers
 * @param hi_reg Register address for HI (MSBs)
 * @param lo_reg Register address for LO (LSBs)
 * @param energy Pointer to ENERGY_VALUES structure
 * @return Status code
 */
int read_and_process_energy(uint16_t hi_reg, uint16_t lo_reg,
			    ENERGY_VALUES *energy)
{
	int status = 0;

	/*Read HI register*/
	status = afe_read_32bit_buff(hi_reg, 1, (uint32_t*)&energy->regHi);
	if (status != 0) return status;

	/*Read LO register*/
	status = afe_read_32bit_buff(lo_reg, 1, (uint32_t*)&energy->regLo);
	if (status != 0) return status;

	/*Combine HI + LO into 45-bit value*/
	energy->currentPeriodRaw = ((int64_t)energy->regHi << 13) | energy->regLo;

	/*Accumulate total*/
	energy->totalRaw += energy->currentPeriodRaw;

	/*Convert to Wh*/
	energy->totalWh = energy->totalRaw *
				     ENERGY_CONVERSION_CONSTANT;

	return status;
}

/**
 * @brief Read all energy types (P, Q, S) for one phase
 * @param phase Pointer to CHANNEL_ENERGY structure
 * @return Status code
 */
int read_phase_energy(PHASE_ENERGY *phase,
		      uint16_t active_hi_reg, uint16_t active_lo_reg,
		      uint16_t reactive_hi_reg, uint16_t reactive_lo_reg,
		      uint16_t apparent_hi_reg, uint16_t apparent_lo_reg)
{
	int status = 0;

	status = read_and_process_energy(
			 active_hi_reg,
			 active_lo_reg,
			 &phase->activeEnergy); /*Active Energy (P)*/
	if (status != 0) return status;

	status = read_and_process_energy(
			 reactive_hi_reg,
			 reactive_lo_reg,
			 &phase->reactiveEnergy); /*Reactive Energy (Q)*/
	if (status != 0) return status;

	status = read_and_process_energy(
			 apparent_hi_reg,
			 apparent_lo_reg,
			 &phase->apparentEnergy); /*Apparent Energy (S)*/
	if (status != 0) return status;

	return status;
}

/**
 * @brief Read all energy data for all 3 phases
 * @param allEnergy Pointer to ALL_CHANNELS_ENERGY structure
 * @return Status code
 */
int read_all_energy(ALL_PHASES_ENERGY *allEnergy)
{
	int status = 0;

	status = read_phase_energy(&allEnergy->phaseA,
				   REG_AWATTHR_HI, REG_AWATTHR_LO,
				   REG_AVARHR_HI, REG_AVARHR_LO,
				   REG_AVAHR_HI, REG_AVAHR_LO);  /* Phase A */
	if (status != 0) return status;

	status = read_phase_energy(&allEnergy->phaseB,
				   REG_BWATTHR_HI, REG_BWATTHR_LO,
				   REG_BVARHR_HI, REG_BVARHR_LO,
				   REG_BVAHR_HI, REG_BVAHR_LO);  /* Phase B */
	if (status != 0) return status;

	status = read_phase_energy(&allEnergy->phaseC,
				   REG_CWATTHR_HI, REG_CWATTHR_LO,
				   REG_CVARHR_HI, REG_CVARHR_LO,
				   REG_CVAHR_HI, REG_CVAHR_LO);  /* Phase C */
	if (status != 0) return status;

	return status;
}