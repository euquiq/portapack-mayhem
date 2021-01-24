/*
 * Copyright (C) 2015 Jared Boone, ShareBrained Technology, Inc.
 * Copyright (C) 2016 Furrtek
 * Copyright (C) 2020 Shao
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "proc_btlerx.hpp"
#include "portapack_shared_memory.hpp"

#include "event_m4.hpp"

//Given a raw RX value from rb_buf, determines bit
bool BTLERxProcessor::get_bit(int16_t index)
{
	return rb_buf[(rb_head + index) % RB_SIZE] > g_threshold;
}

void BTLERxProcessor::execute(const buffer_c8_t &buffer)
{
	if (!configured)
		return;

	// FM demodulation
	const auto decim_0_out = decim_0.execute(buffer, dst_buffer);
	feed_channel_stats(decim_0_out);

	auto audio_oversampled = demod.execute(decim_0_out, work_audio_buffer);

	// Audio signal processing
	for (size_t c = 0; c < audio_oversampled.count; c++)
	{

		rb_head = (++rb_head) % RB_SIZE;		  //rb_head value goes through a circular pattern of 0 - 999 (RB_SIZE = 1000)
		rb_buf[rb_head] = audio_oversampled.p[c]; //if I directly use this, some results can pass crc but not correct.

		if (--skipSamples < 1)
		{
			//Start searching for a valid preamble (01010101 or 10101010)
			g_threshold = 0; //Asuming a preamble, we calculate the mid-value, safest for differentiating bit 1 from 0
			for (int c = 0; c < 8; c++)
				g_threshold += rb_buf[(rb_head + c) % RB_SIZE];

			g_threshold = g_threshold / 8;

			//Verify this byte is a preamble: Counting the transitions from bit 1 to 0 (or viceversa)
			uint8_t transitions = 0;
			for (int c = 0; c < 7; c++)
			{
				if (get_bit(c) != get_bit(c + 1))
				{
					transitions++;
				}
			}

			bool packet_detected = false;
			//If we get 7 transitions, then this first byte is a preamble byte.
			if (transitions == 7)
			{
				uint8_t packet_data[500];
				int packet_length;
				uint32_t packet_crc;
				uint32_t calced_crc;
				uint64_t packet_addr_l;
				uint8_t crc[3];
				uint8_t packet_header_arr[2];

				//Extract the BT LE Access Address (For all advertising packet is uses fixed pattern "0x8E89BED6")
				// packet_addr_l = 0;
				// for (int i = 0; i < 4; i++)
				// {
				// 	bool current_bit;
				// 	uint8_t byte = 0;
				// 	for (int c = 0; c < 8; c++)
				// 	{
				// 		current_bit = get_bit((i + 1) * 8 + c);
				// 		byte |= current_bit << (7 - c);
				// 	}
				// 	uint8_t byte_temp = (uint8_t)(((byte * 0x0802LU & 0x22110LU) | (byte * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16);
				// 	packet_addr_l |= ((uint64_t)byte_temp) << (8 * i);
				// }

				packet_addr_l=0;
				for (int i=0;i<4;i++) 
				{                   
				    bool current_bit;
				    uint8_t byte=0;
				    for (int c=0;c<8;c++)
				    {
				        if (rb_buf[(rb_head + (i+1)*8 + c)%RB_SIZE] > g_threshold)
				            current_bit = true;
				        else
				            current_bit = false;
				        byte |= current_bit << (7-c);
				    }
				    uint8_t byte_temp = (uint8_t) (((byte * 0x0802LU & 0x22110LU) | (byte * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16);
				    packet_addr_l|=((uint64_t)byte_temp)<<(8*i);
				}

				channel_number = 38; //Advertising packets can be TXed thru channels 37, 38, 39

				//Get packet header (2 bytes) First 4 msb bits determine the PDU type
				//For basic bt (present in all bt versions) 0000 ADV_IND  0001 ADV_DIRECT_IND 0010 ADV_NONCONN_IND 0110 ADV_SCAN_IND
				// for (int t = 0; t < 2; t++)
				// {
				// 	bool current_bit;
				// 	uint8_t byte = 0;
				// 	for (int c = 0; c < 8; c++)
				// 	{
				// 		current_bit = get_bit(5 * 8 + t * 8 + c);
				// 		byte |=  current_bit << (7 - c);
				// 	}
				// 	packet_header_arr[t] = byte;
				// }

				for (int t=0;t<2;t++)
				{
				    bool current_bit;
				    uint8_t byte=0;
				    for (int c=0;c<8;c++)
				    {
				        if (rb_buf[(rb_head + 5*8+t*8 + c)%RB_SIZE] > g_threshold)
				            current_bit = true;
				        else
				            current_bit = false;
				        byte |= current_bit << (7-c);
				    }

				    packet_header_arr[t] = byte;
				}

				uint8_t byte_temp2 = (uint8_t)(((channel_number * 0x0802LU & 0x22110LU) | (channel_number * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16);
				uint8_t lfsr_1 = byte_temp2 | 2;
				int header_length = 2;
				int header_counter = 0;
				while (header_length--)
				{
					for (uint8_t i = 0x80; i; i >>= 1)
					{
						if (lfsr_1 & 0x80)
						{
							lfsr_1 ^= 0x11;
							(packet_header_arr[header_counter]) ^= i;
						}
						lfsr_1 <<= 1;
					}
					header_counter = header_counter + 1;
				}

				if (packet_addr_l == 0x8E89BED6)
				{
					uint8_t byte_temp3 = (uint8_t)(((packet_header_arr[1] * 0x0802LU & 0x22110LU) | (packet_header_arr[1] * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16);
					packet_length = byte_temp3 & 0x3F;
				}
				else
				{
					packet_length = 0;
				}

				// for (int t = 0; t < packet_length + 2 + 3; t++)
				// {
				// 	bool current_bit;
				// 	uint8_t byte = 0;
				// 	for (int c = 0; c < 8; c++)
				// 	{
				// 		current_bit = get_bit(5 * 8 + t * 8 + c);
				// 		byte |= current_bit << (7 - c);
				// 	}					
				// 	packet_data[t] = byte;
				// }

				for (int t=0;t<packet_length+2+3;t++)
				{
				    bool current_bit;
				    uint8_t byte=0;
				    for (int c=0;c<8;c++)
				    {
				        if (rb_buf[(rb_head + 5*8+t*8 + c)%RB_SIZE] > g_threshold)
				            current_bit = true;
				        else
				            current_bit = false;
				        byte |= current_bit << (7-c);
				    }

				    packet_data[t] = byte;
				}

				uint8_t byte_temp4 = (uint8_t)(((channel_number * 0x0802LU & 0x22110LU) | (channel_number * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16);
				uint8_t lfsr_2 = byte_temp4 | 2;
				int pdu_crc_length = packet_length + 2 + 3;
				int pdu_crc_counter = 0;
				while (pdu_crc_length--)
				{
					for (uint8_t i = 0x80; i; i >>= 1)
					{
						if (lfsr_2 & 0x80)
						{
							lfsr_2 ^= 0x11;
							(packet_data[pdu_crc_counter]) ^= i;
						}
						lfsr_2 <<= 1;
					}
					pdu_crc_counter = pdu_crc_counter + 1;
				}

				if (packet_addr_l == 0x8E89BED6)
				{
					crc[0] = crc[1] = crc[2] = 0x55;
				}
				else
				{
					crc[0] = crc[1] = crc[2] = 0;
				}

				uint8_t v, t, d, crc_length;
				uint32_t crc_result = 0;
				crc_length = packet_length + 2;
				int counter = 0;
				while (crc_length--)
				{
					uint8_t byte_temp5 = (uint8_t)(((packet_data[counter] * 0x0802LU & 0x22110LU) | (packet_data[counter] * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16);
					d = byte_temp5;
					for (v = 0; v < 8; v++, d >>= 1)
					{
						t = crc[0] >> 7;
						crc[0] <<= 1;
						if (crc[1] & 0x80)
							crc[0] |= 1;
						crc[1] <<= 1;
						if (crc[2] & 0x80)
							crc[1] |= 1;
						crc[2] <<= 1;
						if (t != (d & 1))
						{
							crc[2] ^= 0x5B;
							crc[1] ^= 0x06;
						}
					}
					counter = counter + 1;
				}
				for (v = 0; v < 3; v++)
					crc_result = (crc_result << 8) | crc[v];
				calced_crc = crc_result;

				packet_crc = 0;
				for (int c = 0; c < 3; c++)
					packet_crc = (packet_crc << 8) | packet_data[packet_length + 2 + c];

				if (packet_addr_l == 0x8E89BED6)
				{
					uint8_t mac_data[6];
					int counter = 0;
					for (int i = 7; i >= 2; i--)
					{
						uint8_t byte_temp6 = (uint8_t)(((packet_data[i] * 0x0802LU & 0x22110LU) | (packet_data[i] * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16);
						mac_data[counter] = byte_temp6;
						counter = counter + 1;
					}

					data_message.is_data = false;
					data_message.value = 'A';
					shared_memory.application_queue.push(data_message);

					data_message.is_data = true;
					data_message.value = mac_data[0];
					shared_memory.application_queue.push(data_message);

					data_message.is_data = true;
					data_message.value = mac_data[1];
					shared_memory.application_queue.push(data_message);

					data_message.is_data = true;
					data_message.value = mac_data[2];
					shared_memory.application_queue.push(data_message);

					data_message.is_data = true;
					data_message.value = mac_data[3];
					shared_memory.application_queue.push(data_message);

					data_message.is_data = true;
					data_message.value = mac_data[4];
					shared_memory.application_queue.push(data_message);

					data_message.is_data = true;
					data_message.value = mac_data[5];
					shared_memory.application_queue.push(data_message);

					data_message.is_data = false;
					data_message.value = 'B';
					shared_memory.application_queue.push(data_message);

					packet_detected = true;
				}
				else
					packet_detected = false;
			}

			if (packet_detected)
			{
				skipSamples = 20;
			}
		}
	}
}

void BTLERxProcessor::on_message(const Message *const message)
{
	if (message->id == Message::ID::BTLERxConfigure)
		configure(*reinterpret_cast<const BTLERxConfigureMessage *>(message));
}

void BTLERxProcessor::configure(const BTLERxConfigureMessage &message)
{
	decim_0.configure(taps_200k_wfm_decim_0.taps, 33554432);
	decim_1.configure(taps_200k_wfm_decim_1.taps, 131072);
	demod.configure(audio_fs, 5000);

	configured = true;
}

int main()
{
	EventDispatcher event_dispatcher{std::make_unique<BTLERxProcessor>()};
	event_dispatcher.run();
	return 0;
}