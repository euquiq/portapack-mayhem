/*
 * Copyright (C) 2015 Jared Boone, ShareBrained Technology, Inc.
 * Copyright (C) 2016 Furrtek
 * Copyright (C) 2020 Shao
 * Copyright (C) 2021 We The People ...
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

//	Note from euquiq: This code, when introduced in Portapack, was taken from:
// https://github.com/DesignSparkrs/sdr-ble-demo/blob/master/references/NRF24-BTLE-Decoder/nrf24-btle-decoder.c

#include "proc_btlerx.hpp"
#include "portapack_shared_memory.hpp"

#include "event_m4.hpp"

//Given a raw RX value from rb_buf, determines bit (Quantize)
bool BTLERxProcessor::get_bit(uint16_t index)
{
	return rb_buf[(rb_head + index) % RB_SIZE] > g_threshold;
}

//BTLE transmits the bits in reversed order
uint8_t BTLERxProcessor::SwapBits(uint8_t a)
{
	return (uint8_t) (((a * 0x0802LU & 0x22110LU) | (a * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16);
}

//Un-Whiten (descramble) BTLE packet using channel value. Whitening is applied to the payload and the CRC
void BTLERxProcessor::BTLEWhiten(uint8_t* data, uint8_t len, uint8_t chan)
{
	uint8_t  i;
	uint8_t lfsr = SwapBits(chan) | 2;
	while(len--)
	{
		for(i = 0x80; i; i >>= 1)
		{
			if(lfsr & 0x80)
			{
				lfsr ^= 0x11;
				(*data) ^= i;
			}
			lfsr <<= 1;
		}
		data++;
	}
}

//Calcualte CHKSUM CRC24 for BTLE
uint32_t BTLERxProcessor::BTLECrc(const uint8_t *data, uint8_t len, uint8_t *dst)
{
	uint8_t v, t, d;
	uint32_t crc = 0;
	while (len--)
	{
		d = SwapBits(*data++);
		for (v = 0; v < 8; v++, d >>= 1)
		{
			t = dst[0] >> 7;

			dst[0] <<= 1;
			if (dst[1] & 0x80)
				dst[0] |= 1;
			dst[1] <<= 1;
			if (dst[2] & 0x80)
				dst[1] |= 1;
			dst[2] <<= 1;

			if (t != (d & 1))
			{
				dst[2] ^= 0x5B;
				dst[1] ^= 0x06;
			}
		}
	}
	for (v = 0; v < 3; v++)
		crc = (crc << 8) | dst[v];
	return crc;
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
			//PREAMBLE (01010101 or 10101010)
			g_threshold = 0; //Asuming a preamble, calculate mid-value, separating bit 1 from 0
			for (int z = 0; z < 8; z++)
				g_threshold += rb_buf[(rb_head + z) % RB_SIZE];

			g_threshold = g_threshold / 8;

			uint8_t transitions = 0; 	//Verify this byte is a preamble: Counting the transitions from bit 1 to 0 (or viceversa)
			for (int z = 0; z < 8; z++) //Count 8 transitions (7 for first byte and one more transitioning against first bit of next byte)
			{
				if (get_bit(z) != get_bit(z + 1))
					transitions++;
			}

			bool packet_detected = false;	
			if (transitions == 8) //This first byte is a preamble byte.
			{
				uint8_t packet_data[260];
				int packet_length;
				uint32_t packet_crc;
				uint32_t calced_crc;
				uint64_t packet_addr_l;
				uint8_t crc[3];
				uint8_t packet_header_arr[2];

				// GET ADVERTISING BT LE Access Address (4 bytes)
				packet_addr_l = 0;				
				for (int i = 0; i < 4; i++)
				{
					uint8_t byte = 0;
					for (int z = 0; z < 8; z++) 
						byte |= get_bit(8 + (i * 8) + z) << (7 - z);  //Access starts in rb_head + 8, 4 bytes

					packet_addr_l |= ((uint64_t)SwapBits(byte)) << (8 * i);
				}

				// ADVERTISING PACKET (uses fixed pattern "0x8E89BED6")
				if (packet_addr_l == 0x8E89BED6)
				{
					//Get PDU HEADER (2 bytes) First 4 msb bits determine the PDU type
					//Header is: 4 bits for PDU type (see above), 1 bit for RFU, 1 bit for ChSel, 1 bit for TxAdd, 1 bit for RX Add and 8 bits FOR PACKET LENGTH
					//For basic bt (present in all bt versions) 0000 ADV_IND  0001 ADV_DIRECT_IND 0010 ADV_NONCONN_IND 0110 ADV_SCAN_IND
					
					for (int t = 0; t < 2; t++)
					{
						//uint8_t byte = 0;
						for (int z = 0; z < 8; z++)
							packet_header_arr[t] |=  get_bit(40 + (t * 8) + z) << (7 - z);  //PDU HEADER starts in rb_head + (8 * 5) (1 byte Preamble + 4 bytes Address, 40 bits)

						//packet_header_arr[t] = byte;
					}

					BTLEWhiten(packet_header_arr, 2, 38); //Clear whitening on packet header. Advertising packets can be TXed thru channels 37, 38, 39
					packet_length = SwapBits(packet_header_arr[1]) & 0x3F;  //... So we can get the PAYLOAD LENGTH. This might be WRONG ? ble 5.x uses all 8 bits
					
					//PDU, PAYLOAD: Get the packet DATA 
					for (int t = 0; t < packet_length + 2 + 3; t++) //3 bytes for CRC are just after the payload
					{
						for (int z = 0; z < 8; z++)
							packet_data[t] |= get_bit(40 + (t * 8) + z) << (7 - z); //Now get the whole PDU PAYLOAD
					}

					BTLEWhiten(packet_data, packet_length + 2 + 3, 38); //DATA UNWHITENING to entire PAYLOAD

					//------------------------
					//
					// Problem: CRC does not check at any time
					// and Bluetooth LE "MAC" address is wrong (garbage, always).
					// I tried to skew bits forwards and backwards, re-chechking everything.
					// Also tried to increment and decrement g_threshold by 10% steps
					// Also tried changing packet_length from 6 to 250 recalculating CRC at each byte size ...
					// With no luck.
					//
					//Its as if after the Advertising Address, the whole header / Payload PDU is "garbage"
					//
					//------------------------


					// data_message.is_data = false;
					// data_message.value = 'A';
					// shared_memory.application_queue.push(data_message);

					//CRC CALCULATION
					crc[0] = crc[1] = crc[2] = 0x55; //ADvertisement packet checks into these values.
					calced_crc = BTLECrc(packet_data, packet_length + 2, crc);


					 unsigned char *bytes1 = reinterpret_cast<unsigned char*>(&calced_crc);

					// for (int z = 0; z < 3; z++)
					// {
					// 	data_message.is_data = true;
					// 	data_message.value = (bytes1[z]);
					// 	shared_memory.application_queue.push(data_message);
					// }



					packet_crc = 0;
					for (int z = 0; z < 3; z++)
					{
					// 	data_message.is_data = true;
					// 	data_message.value = packet_data[packet_length + 2 + z];
					// 	shared_memory.application_queue.push(data_message);

					 	packet_crc = (packet_crc << 8) | packet_data[packet_length + 2 + z];
					}

					//  unsigned char *bytes2 = reinterpret_cast<unsigned char*>(&packet_crc);
					// for (int z = 0; z < 3; z++)
					// {
					// 	data_message.is_data = true;
					// 	data_message.value = (bytes2[z]);
					// 	shared_memory.application_queue.push(data_message);
					// }


					//Confirmed adv packet thru CRC:
					//if (packet_crc == calced_crc){
						//PDU - PAYLOAD DATA: bytes 2 to 7 of ADVERTISER PDU used for BLUETOOTH DEVICE ADDRESS (BDA) like MAC address, stored in "little endian", backwards
						uint8_t mac_data[6];
						for (int i = 7; i >=2; i--)
							mac_data[7-i] = SwapBits(packet_data[i]);


						data_message.is_data = false;
						data_message.value = 'A';
						shared_memory.application_queue.push(data_message);



						// for (int t = 2; t < packet_length + 2; t++)
						// {
						// data_message.is_data = true;
						// data_message.value = SwapBits(packet_data[t]);
						// shared_memory.application_queue.push(data_message);
						// }

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

						// data_message.is_data = true;
						// data_message.value = packet_length;
						// shared_memory.application_queue.push(data_message);

						data_message.is_data = false;
						data_message.value = 'B';
						shared_memory.application_queue.push(data_message);

						packet_detected = true;
					//}

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