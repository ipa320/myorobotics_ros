 /*
 * Copyright (c) 2013, Shadow Robot Company, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */

/**
 * @file   sr_spi.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  Driver for the RoNeX mk2 General I/O module.
 **/

#include <ros_ethercat_model/robot_state.hpp>
#include <ros_ethercat_model/joint.hpp>
#include <ros_ethercat_hardware/ethercat_hardware.h>

#include <sstream>
#include <iomanip>
#include <boost/lexical_cast.hpp>
#include <math.h>
#include <string>

#include <sr_ronex_drivers/ronex_utils.hpp>

#include <myo_interface/myoMuscleJointInterface.h>
#include "myo_control/sr_spi_myo.hpp"

PLUGINLIB_EXPORT_CLASS(SrSPIMyo, EthercatDevice);

using boost::lexical_cast;

const char SrSPIMyo::product_alias_[] = PRODUCT_NAME;

SrSPIMyo::SrSPIMyo() :
	node_("~"), cycle_count_(0)
{}

SrSPIMyo::~SrSPIMyo()
{
	// remove parameters from server
	string device_id = "/ronex/devices/" + lexical_cast<string>(parameter_id_);
	ros::param::del(device_id);

	string controller_name = "/ronex_" + serial_number_ + "_passthrough";
	ros::param::del(controller_name);

	string spi_device_name = "/ronex/spi/" + serial_number_;
	ros::param::del(spi_device_name);
}

void SrSPIMyo::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
	sh_ = sh;
	serial_number_ = ronex::get_serial_number(sh);

	// get the alias from the parameter server if it exists
	string path_to_alias, alias;
	path_to_alias = "/ronex/mapping/" + serial_number_;
	if ( ros::param::get(path_to_alias, alias))
	{
		ronex_id_ = alias;
	}
	else
	{
		// no alias found, using the serial number directly.
		ronex_id_ = serial_number_;
	}

	device_name_ = ronex::build_name(product_alias_, ronex_id_);

	command_base_  = start_address;
	command_size_  = COMMAND_ARRAY_SIZE_BYTES;

	start_address += command_size_;
	status_base_   = start_address;
	status_size_   = STATUS_ARRAY_SIZE_BYTES;

	start_address += status_size_;

	// ETHERCAT_COMMAND_DATA
	//
	// This is for data going TO the board
	//

	if ( (PROTOCOL_TYPE) == (EC_BUFFERED) )
	{
		ROS_INFO("Using EC_BUFFERED");
	}
	else if ( (PROTOCOL_TYPE) == (EC_QUEUED) )
	{
		ROS_INFO("Using EC_QUEUED");
	}

	ROS_INFO("First FMMU (command) : Logical address: 0x%08X ; size: %3d bytes ; ET1200 address: 0x%08X", command_base_,
					 command_size_, static_cast<int>(COMMAND_ADDRESS) );
	EC_FMMU *commandFMMU = new EC_FMMU( command_base_,    // Logical Start Address    (in ROS address space?)
																			command_size_,
																			0x00,             // Logical Start Bit
																			0x07,             // Logical End Bit
																			COMMAND_ADDRESS,  // Physical Start Address   (in ET1200 address space?)
																			0x00,             // Physical Start Bit
																			false,            // Read Enable
																			true,             // Write Enable
																			true);              // Channel Enable



	// WARNING!!!
	// We are leaving (command_size_ * 4) bytes in the physical memory of the device, but strictly we only need to
	// leave (command_size_ * 3). This change should be done in the firmware as well, otherwise it won't work.
	// This triple buffer is needed in the ethercat devices to work in EC_BUFFERED mode (in opposition to the other mode
	// EC_QUEUED, the so called mailbox mode)

	// ETHERCAT_STATUS_DATA
	//
	// This is for data coming FROM the board
	//
	ROS_INFO("Second FMMU (status) : Logical address: 0x%08X ; size: %3d bytes ; ET1200 address: 0x%08X", status_base_,
					 status_size_, static_cast<int>(STATUS_ADDRESS) );
	EC_FMMU *statusFMMU = new EC_FMMU(  status_base_,
																			status_size_,
																			0x00,
																			0x07,
																			STATUS_ADDRESS,
																			0x00,
																			true,
																			false,
																			true);


	EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);

	(*fmmu)[0] = *commandFMMU;
	(*fmmu)[1] = *statusFMMU;

	sh->set_fmmu_config(fmmu);

	EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(2);

// SyncMan takes the physical address
	(*pd)[0] = EC_SyncMan(COMMAND_ADDRESS, command_size_, PROTOCOL_TYPE, EC_WRITTEN_FROM_MASTER);
	(*pd)[1] = EC_SyncMan(STATUS_ADDRESS,  status_size_,  PROTOCOL_TYPE);


	(*pd)[0].ChannelEnable = true;
	(*pd)[0].ALEventEnable = true;
	(*pd)[0].WriteEvent    = true;

	(*pd)[1].ChannelEnable = true;

	sh->set_pd_config(pd);

	ROS_INFO("Finished constructing the SrSPIMyo driver");
}

int SrSPIMyo::initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
	digital_commands_ = 0;
	ROS_INFO("Device #%02d: Product code: %u (%#010X) , Serial #: %u (%#010X)",
						sh_->get_ring_position(),
						sh_->get_product_code(),
						sh_->get_product_code(),
						sh_->get_serial(),
						sh_->get_serial());

	device_offset_ = sh_->get_ring_position();

	// add the RoNeX SPI module to the hw interface
	ros_ethercat_model::RobotState *robot_state = static_cast<ros_ethercat_model::RobotState*>(hw);
	robot_state->custom_hws_.insert(device_name_, new ronex::SPI());
	spi_ = static_cast<ronex::SPI*>(robot_state->getCustomHW(device_name_));

	build_topics_();

	joint_states_[0] = robot_state->getJointState("muscle0");
	joint_states_[1] = robot_state->getJointState("muscle1");
	joint_states_[2] = robot_state->getJointState("muscle2");
	joint_states_[3] = robot_state->getJointState("muscle3");

	ROS_INFO_STREAM("Adding a SPI RoNeX module to the hardware interface: " << device_name_);
	// Using the name of the ronex to prefix the state topic

	return 0;
}

void SrSPIMyo::packCommand(unsigned char *buffer, bool halt, bool reset)
{
	RONEX_COMMAND_02000002* command = reinterpret_cast<RONEX_COMMAND_02000002*>(buffer);

	// Set Command-Type
	command->command_type = RONEX_COMMAND_02000002_COMMAND_TYPE_NORMAL;
	// Configure SlaveSelect on all 4 SPI-Ports
	int16u dio = 0;
	if (joint_states_[0]->digitalOut_) dio |= PIN_OUTPUT_STATE_DIO_0;
	if (joint_states_[1]->digitalOut_) dio |= PIN_OUTPUT_STATE_DIO_1;
	if (joint_states_[2]->digitalOut_) dio |= PIN_OUTPUT_STATE_DIO_2;
	if (joint_states_[3]->digitalOut_) dio |= PIN_OUTPUT_STATE_DIO_3;

	command->pin_output_states_pre = dio;
	command->pin_output_states_post = dio | PIN_OUTPUT_STATE_CS_0 | PIN_OUTPUT_STATE_CS_1 | PIN_OUTPUT_STATE_CS_2 | PIN_OUTPUT_STATE_CS_3;

	for (size_t spi_index = 0; spi_index < NUM_SPI_OUTPUTS; ++spi_index)
	{
		// 64 MHz / 16 = 4 MHz
		command->spi_out[spi_index].clock_divider = 16;
		// Clock normally low, sample on falling edge 
		command->spi_out[spi_index].SPI_config = SPI_CONFIG_MODE_01;
		command->spi_out[spi_index].inter_byte_gap = 4;//0;
		// Transmition length
		command->spi_out[spi_index].num_bytes = 24;

		int dutyCycle = joint_states_[spi_index]->commanded_effort_; // TODO
		// Init SPI-data
		command->spi_out[spi_index].data_bytes[0] = 0x80;
		command->spi_out[spi_index].data_bytes[1] = 0x00;
		command->spi_out[spi_index].data_bytes[2] = (dutyCycle >> 8) & 0x7F;
		command->spi_out[spi_index].data_bytes[3] = dutyCycle;
		// Fill rest of SPI-frame with 0
		for ( size_t i = 4; i < SPI_TRANSACTION_MAX_SIZE; ++i )
		{
			command->spi_out[spi_index].data_bytes[i] = 0;
		}

		if ( command->spi_out[spi_index].num_bytes != 0)
		{
			ostringstream ss;
			ss << "SPI out [" << spi_index << "]: Sending non null command("
			<<static_cast<unsigned int>(command->spi_out[spi_index].num_bytes) << "): -> ";

			for (unsigned int i = 0; i < static_cast<unsigned int>(command->spi_out[spi_index].num_bytes); ++i)
				ss << static_cast<int>(command->spi_out[spi_index].data_bytes[i]) << ",";

			ROS_DEBUG_STREAM("" << ss.str());
		}
	}
}

bool SrSPIMyo::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
	RONEX_STATUS_02000002* status_data = reinterpret_cast<RONEX_STATUS_02000002 *>(this_buffer+  command_size_);

	// Checking that the received command type matches one of the valid commands
	// (we could check for the one that was used in the command but this is easier).
	// The ronex firmware will answer with whatever command type we send.
	// The purpose of this is to filter those status_data structures that come filled with zeros due to the jitter
	// in the realtime loop. The jitter causes that the host tries to read the status when the microcontroller in the
	// ronex module has not finished writing it to memory yet.
	if ( status_data->command_type == RONEX_COMMAND_02000002_COMMAND_TYPE_NORMAL)
	{
		// copying the status data to the HW interface
		spi_->state_->command_type = status_data->command_type;

		for (size_t sampling = 0; sampling < NUM_DIO_SAMPLES; ++sampling)
		{
			spi_->state_->info_type.status_data.pin_input_states_DIO[sampling] =
							status_data->info_type.status_data.pin_input_states_DIO[sampling];
		}

		for (size_t spi_index=0; spi_index < NUM_SPI_OUTPUTS; ++spi_index)
		{
			int8u *data = status_data->info_type.status_data.spi_in[spi_index].data_bytes;
			// TODO decode sensor values
			int position = (data[8] << 24) + (data[9] << 16) + (data[10] << 8) + data[11];
			short velocity = (data[12] << 8) + data[13];
			short current = (data[14] << 8) + data[15];
			short displacement = (data[16] << 8) + data[17];
			short sensor1 = (data[18] << 8) + data[19];
			short sensor2 = (data[20] << 8) + data[21];
			joint_states_[spi_index]->position_ = position;
			joint_states_[spi_index]->velocity_ = velocity;
			joint_states_[spi_index]->effort_ = current;
			joint_states_[spi_index]->displacement_ = displacement;
			for (int i = 0; i < 6; i++)
			{
				joint_states_[spi_index]->analogIN_[i] = status_data->info_type.status_data.analogue_in[i];
			}
		}

		for (size_t analogue_index = 0 ; analogue_index < NUM_ANALOGUE_INPUTS ; ++analogue_index)
		{
			spi_->state_->info_type.status_data.analogue_in[analogue_index] =
							status_data->info_type.status_data.analogue_in[analogue_index];
		}
	}
	else if ( status_data->command_type == RONEX_COMMAND_02000002_COMMAND_TYPE_CONFIG_INFO)
	{
		// This command type is not used for the time being. For future expansion.
	}

	// publishing at 100Hz
	if (cycle_count_ > 9)
	{
		state_msg_.header.stamp = ros::Time::now();

		state_msg_.command_type = spi_->state_->command_type;

		for (size_t sampling = 0; sampling < NUM_DIO_SAMPLES; ++sampling)
		{
			state_msg_.pin_input_states_DIO[sampling] = spi_->state_->info_type.status_data.pin_input_states_DIO[sampling];
			state_msg_.pin_input_states_SOMI[sampling] = spi_->state_->info_type.status_data.pin_input_states_SOMI[sampling];
		}

		for (size_t spi_index=0; spi_index < NUM_SPI_OUTPUTS; ++spi_index)
		{
			for (size_t i = 0; i < SPI_TRANSACTION_MAX_SIZE; ++i)
				state_msg_.spi_in[spi_index].data[i] = spi_->state_->info_type.status_data.spi_in[spi_index].data_bytes[i];
		}

		for (size_t analogue_index = 0 ; analogue_index < NUM_ANALOGUE_INPUTS ; ++analogue_index)
		{
			state_msg_.analogue_in[analogue_index] = spi_->state_->info_type.status_data.analogue_in[analogue_index];
		}

		// publish
		if ( state_publisher_->trylock() )
		{
			state_publisher_->msg_ = state_msg_;
			state_publisher_->unlockAndPublish();
		}

		cycle_count_ = 0;
	}

	cycle_count_++;
	return true;
}

void SrSPIMyo::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
	d.name = device_name_;
	d.summary(d.OK, "OK");
	d.hardware_id = serial_number_;

	d.clear();

	// @TODO add more diagnostics
}

void SrSPIMyo::build_topics_()
{
	// loading everything into the parameter server
	parameter_id_ = ronex::get_ronex_param_id("");
	ostringstream param_path, tmp_param;
	param_path << "/ronex/devices/" << parameter_id_ << "/";
	tmp_param << ronex::get_product_code(sh_);
	ros::param::set(param_path.str() + "product_id", tmp_param.str());
	ros::param::set(param_path.str() + "product_name", product_alias_);
	ros::param::set(param_path.str() + "ronex_id", ronex_id_);

	// the device is stored using path as the key in the CustomHW map
	ros::param::set(param_path.str() + "path", device_name_);
	ros::param::set(param_path.str() + "serial", serial_number_);

	// Advertising the realtime state publisher
	state_publisher_.reset(
					new realtime_tools::RealtimePublisher<sr_ronex_msgs::SPIState>(node_, device_name_ + "/state", 1));
}

/* For the emacs weenies in the crowd.
	 Local Variables:
	 c-basic-offset: 2
	 End:
*/
