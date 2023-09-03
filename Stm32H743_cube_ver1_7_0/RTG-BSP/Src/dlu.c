/*
 * dlu.c
 *
 *  Created on: 10 Apr 2022
 *      Author: itzhaki
 */

#include <dlu.h>
#include <timer.h>

//ADT_CBIT_receive_message ADT_CBIT_rec_message =
//{ 0 };
//ADT_PBIT_receive_message ADT_PBIT_rec_message =
//{ 0 };

void DLU_pbit_buld_struct(ADT_PBIT_receive_message *buff,
		ADT_PBIT_send_message *dist)
{
	dist->TimeTag = GetTimeTag;

	// WILL BE SET valid function ------------------------------------------
	dist->PassFaild.error8 = 0; // Clear errors
	dist->PassFaild.error.ADT_Flash_memories_Failure =
			buff->ADT_Flash_memories_Failure;
	dist->PassFaild.error.ADT_DRAMs_Failure = buff->ADT_DRAMs_Failure;
	dist->PassFaild.error.ADT_DSP_Bit_Failure = buff->ADT_DSP_Bit_Failure;
	//==== is change >> 1 PBIT Didn't done , yet 0 PBIT Done
	dist->PassFaild.error.PBIT_Done = !buff->PBIT_Done;
	dist->PassFaild.error.FW_Main_version_Fail = buff->FW_Main_version_Fail;
	dist->PassFaild.error.DSP_SW_Main_version_Fail =
			buff->DSP_SW_Main_version_Fail;
	dist->PassFaild.error.DSP_SW_images_Fail = buff->DSP_SW_images_Fail;
}

void DLU_cbit_buld_struct(ADT_CBIT_receive_message *buff,
		ADT_CBIT_send_message *dist)
{
	dist->TimeTag = GetTimeTag;

// WILL BE SET valid function ------------------------------------------
	dist->PassFaild.error16 = 0; // clear Passfail
	dist->PassFaild.error.ADT_SP4T_failure = buff->ADT_SP4T_failure;
	dist->PassFaild.error.internal_DC_voltage_failure =
			buff->internal_DC_voltage_failure;
	dist->PassFaild.error.digital_section_failure =
			buff->digital_section_failure;
	dist->PassFaild.error.RF_failure = buff->RF_failure;
	dist->PassFaild.error.synthesizers_lock_detect =
			buff->synthesizers_lock_detect;
	dist->PassFaild.error.pedestal_failure = buff->pedestal_failure;
	dist->PassFaild.error.PA_over_current_failure =
			buff->PA_over_current_failure;
	dist->PassFaild.error.Rx_input_over_power = buff->Rx_input_over_power;
	dist->PassFaild.error.digital_section_over_heat =
			buff->digital_section_over_heat;
	dist->PassFaild.error.RF_section_over_heat = buff->RF_section_over_heat;
	dist->PassFaild.error.system_startUp = buff->system_startUp;
	dist->PassFaild.error.PA_power_output_low = buff->PA_power_output_low;
	dist->PassFaild.error.PA_return_power_failure =
			buff->PA_return_power_failure;
	dist->PA_power_output = buff->PA_power_output;
	dist->PA_return_power = buff->PA_return_power;
}
