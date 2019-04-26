#include "odrive_cpp_ros/odrive_cpp_ros.h"
#include "odrive_cpp_ros/odrive_endpoints.h"
#include <algorithm>
#include <cstring>
using namespace odrive;

ODriveDriver::ODriveDriver(const std::string* odrive_serial_numbers,
               const uint8_t num_odrives,
               const std::string* motor_to_odrive_serial_number_map,
               const uint8_t* motor_index_map,
               const uint8_t num_motors) {
   
    // read settings
    num_odrives_ = num_odrives;
    num_motors_ = num_motors;

    // saved for use between creation and init
    odrive_serial_numbers_ = new std::string[num_odrives]();
    for (uint8_t i = 0; i < num_odrives; ++i) {
        odrive_serial_numbers_[i].assign(odrive_serial_numbers[i]);
    }

    motor_to_odrive_serial_number_map_ = new std::string[num_motors]();
    motor_index_map_ = new uint8_t[num_motors]();
    for (uint8_t i = 0; i < num_motors; ++i) {
        motor_to_odrive_serial_number_map_[i].assign(motor_to_odrive_serial_number_map[i]);
        motor_index_map_[i] = motor_index_map[i];
    }

    motor_to_odrive_handle_index_ = NULL;
    odrive_handles_ = NULL;
    libusb_context_ = NULL;
}

ODriveDriver::~ODriveDriver() {
    if (motor_to_odrive_handle_index_) {
        for (uint8_t i = 0; i < num_odrives_; ++i) {
            if (odrive_handles_[i]) {
                int result = libusb_release_interface(odrive_handles_[i], 0);
                if (result != LIBUSB_SUCCESS) {
                    std::cerr << "Error calling libusb_release_interface on odrive `" << odrive_serial_numbers_[i] << "`: " << result << " - " << libusb_error_name(result) << std::endl;
                }

                libusb_close(odrive_handles_[i]);
                odrive_handles_[i] = NULL;
            }
        }
    }

    if (motor_index_map_) delete [] motor_index_map_;
    if (odrive_serial_numbers_) delete [] odrive_serial_numbers_;
    if (motor_to_odrive_serial_number_map_) delete [] motor_to_odrive_serial_number_map_;

    // usb
    if (odrive_handles_) delete [] odrive_handles_;
    if (motor_to_odrive_handle_index_) delete [] motor_to_odrive_handle_index_;
    if (libusb_context_) { libusb_exit(libusb_context_); }
}

int ODriveDriver::init() {
    if (libusb_context_ != NULL) {
      std::cerr << "ODrive SDK's init function has been called twice." << std::endl;
      return ODRIVE_SDK_NOT_INITIALIZED;
    }

    outbound_seq_no_ = 0;
    // for USB
    int result = libusb_init(&libusb_context_);
    if (result != LIBUSB_SUCCESS) {
        return result; // error message should have been printed before
    }

    odrive_handles_ = new libusb_device_handle*[num_odrives_] {NULL};
    result = initUSBHandlesBySNs();
    if (result != ODRIVE_SDK_COMM_SUCCESS) {
        return result; // error message should have been printed before
    }
    for (uint8_t i = 0; i < num_odrives_; ++i) {
        if (!odrive_handles_[i]) {
            return ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND;
        }
    }

    // for each motor, reference to the index of the handle
    motor_to_odrive_handle_index_ = new uint8_t[num_motors_]();
    for (uint8_t i = 0; i < num_motors_; ++i) {
        long sn_index = std::distance(odrive_serial_numbers_, std::find(odrive_serial_numbers_, odrive_serial_numbers_ + num_odrives_, motor_to_odrive_serial_number_map_[i]));
        if (sn_index < 0 || sn_index >= num_motors_) {
            return ODRIVE_SDK_SERIAL_NUMBER_MAP_INVALID;
        }
        motor_to_odrive_handle_index_[i] = (uint8_t) sn_index;
    }

    return ODRIVE_SDK_COMM_SUCCESS;
}

int ODriveDriver::getFloat(int motor_index, float &param, int endpoint_id) {
    if (! motor_to_odrive_handle_index_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int axis_offset = (motor_index_map_[motor_index] == 1) ? per_axis_offset : 0;
    int cmd = endpoint_id + axis_offset;

    uint8_t handle_index = motor_to_odrive_handle_index_[motor_index];

    int result = odriveEndpointGetFloat(odrive_handles_[handle_index], cmd, param);
    if (result != LIBUSB_SUCCESS) {
        std::cerr << "Couldn't send `" << std::to_string(cmd) << "` to '" << odrive_serial_numbers_[handle_index] << "': `" << result << "` (see prior error message)" << std::endl;
        return ODRIVE_SDK_UNEXPECTED_RESPONSE;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}


int ODriveDriver::getInt(int motor_index, int &param, int endpoint_id) {
    if (! motor_to_odrive_handle_index_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int axis_offset = (motor_index_map_[motor_index] == 1) ? per_axis_offset : 0;
    int cmd = endpoint_id + axis_offset;

    uint8_t handle_index = motor_to_odrive_handle_index_[motor_index];

    int result = odriveEndpointGetInt(odrive_handles_[handle_index], cmd, param);
    if (result != LIBUSB_SUCCESS) {
        std::cerr << "Couldn't send `" << std::to_string(cmd) << "` to '" << odrive_serial_numbers_[handle_index] << "': `" << result << "` (see prior error message)" << std::endl;
        return ODRIVE_SDK_UNEXPECTED_RESPONSE;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int ODriveDriver::getMotorSpeed(int motor_index, float &motor_speed) {
    getFloat(motor_index, motor_speed, AXIS__ENCODER__VEL_ESTIMATE);
    std::cout << "Motor speed: " << motor_index << " : " << motor_speed << std::endl;
}

int ODriveDriver::getMotorPosition(int motor_index, int &motor_position) {
    getInt(motor_index, motor_position, AXIS__ENCODER__COUNT_IN_CPR);
    std::cout << "Motor pos: " << motor_index << " : " << motor_position;
}


int ODriveDriver::setMotorSpeed(int motor_index, float motor_speed) {
    if (! motor_to_odrive_handle_index_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }
    
    int axis_offset = (motor_index_map_[motor_index] == 1) ? per_axis_offset : 0;
    int cmd = AXIS__CONTROLLER__VEL_SETPOINT + axis_offset;
    
    uint8_t handle_index = motor_to_odrive_handle_index_[motor_index];

    int result = odriveEndpointSetFloat(odrive_handles_[handle_index], cmd, motor_speed);
    
    if (result != LIBUSB_SUCCESS) {
        std::cerr << "Couldn't send `" << std::to_string(cmd) << "` to '" << odrive_serial_numbers_[handle_index] << "': `" << result << "` (see prior error message)" << std::endl;
        return ODRIVE_SDK_UNEXPECTED_RESPONSE;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int ODriveDriver::setMotorSpeeds(float* motor_speeds) {
    for (uint8_t i = 0; i < num_motors_; ++i) {
        int result = setMotorSpeed(i, motor_speeds[i]);
        if (result != ODRIVE_SDK_COMM_SUCCESS) {
            std::cout << "Setting speed for motor #" << i << " failed. Result: " << result << std::endl;
            return result;
        }
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int ODriveDriver::readCurrentMotorPosition(int motor_index, int &motor_position) {
    if (! motor_to_odrive_handle_index_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int axis_offset = (motor_index_map_[motor_index] == 1) ? per_axis_offset : 0;
    int cmd = AXIS__ENCODER__COUNT_IN_CPR + axis_offset;
    
    uint8_t handle_index = motor_to_odrive_handle_index_[motor_index];
    int ticks_read;
    int result = odriveEndpointGetInt(odrive_handles_[handle_index], cmd, ticks_read);
    
    if (result != LIBUSB_SUCCESS) {
        std::cerr << "Couldn't send `" << std::to_string(cmd) << "` to '" << odrive_serial_numbers_[handle_index] << "': `" << result << "` (see prior error message)" << std::endl;
        return ODRIVE_SDK_UNEXPECTED_RESPONSE;
    }
    motor_position = ticks_read;
    return ODRIVE_SDK_COMM_SUCCESS;
}


int ODriveDriver::readCurrentMotorPositions(int* axes_positions) {
    int motor_position;
    for (uint8_t i = 0; i < num_motors_; ++i) {
        int result = readCurrentMotorPosition(i, motor_position);
        if (result != ODRIVE_SDK_COMM_SUCCESS) {
            std::cout << "Reading position from motor #" << i << " failed. Result: " << result << std::endl;
            return result;
        }
        axes_positions[i] = motor_position;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int ODriveDriver::checkErrors(uint8_t* error_codes_array) {
    if (! motor_to_odrive_handle_index_) {
        return ODRIVE_SDK_NOT_INITIALIZED;
    }

    int cmd;
    int axis_offset;
    for (uint8_t i = 0; i < num_motors_; ++i) {
        axis_offset = (motor_index_map_[i] == 1) ? per_axis_offset : 0;
        cmd = AXIS__ERROR + axis_offset;
        
        uint8_t handle_index = motor_to_odrive_handle_index_[i];
        
        uint8_t motor_error_output;
        int result = odriveEndpointGetUInt8(odrive_handles_[handle_index], cmd, motor_error_output);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "CppSdk::checkErrors couldn't send `" << std::to_string(cmd) << "` to '" << odrive_serial_numbers_[handle_index] << "': `" << result << "` (see prior error message)" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }
        error_codes_array[i] = motor_error_output;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}


int ODriveDriver::initUSBHandlesBySNs() {
    libusb_device ** usb_device_list;
    ssize_t device_count = libusb_get_device_list(libusb_context_, &usb_device_list);
    if (device_count <= 0) {
        std::cerr << "Could not call libusb_get_device_list: " << device_count << " - " << libusb_error_name(device_count) << std::endl;
        return device_count;
    }

    for (size_t i = 0; i < device_count; ++i) {
        libusb_device *device = usb_device_list[i];
        libusb_device_descriptor desc = {0};

        int result = libusb_get_device_descriptor(device, &desc);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "Could not call libusb_get_device_descriptor: " << result << " - " << libusb_error_name(result) << std::endl;
        }

        if (desc.idVendor == ODRIVE_SDK_USB_VENDORID && ( desc.idProduct == ODRIVE_SDK_USB_PRODUCTID_0 ||  desc.idProduct == ODRIVE_SDK_USB_PRODUCTID_1 )) {
            libusb_device_handle *device_handle;
            result = libusb_open(device, &device_handle);
            if (result != LIBUSB_SUCCESS) {
                std::cerr << "Could not call libusb_open: " << result << " - " << libusb_error_name(result) << std::endl;
            } else if (libusb_kernel_driver_active(device_handle, 0) && ( (result = libusb_detach_kernel_driver(device_handle, 0)) != LIBUSB_SUCCESS )) { // detach kernel driver if necessary
                std::cerr << "Could not call libusb_detach_kernel_driver: " << result << " - " << libusb_error_name(result) << std::endl;
            } else if ( (result = libusb_claim_interface(device_handle, 0)) !=  LIBUSB_SUCCESS ) {
              std::cerr << "Could not call libusb_claim_interface: " << result << " - " << libusb_error_name(result) << ": " << strerror(errno) << std::endl;
              libusb_close(device_handle);
            } else {
                bool attached_to_handle = false;
                uint64_t read_serial_number;
                int result = odriveEndpointGetUInt64(device_handle, ODRIVE_SDK_SERIAL_NUMBER_CMD, read_serial_number);
                if (result != LIBUSB_SUCCESS) {
                    std::cerr << "Couldn't send `" << std::to_string(ODRIVE_SDK_SERIAL_NUMBER_CMD) << "` to '0d" << std::to_string(desc.idVendor) << ":" << std::to_string(desc.idProduct) << "': `" << result << " - " << libusb_error_name(result) << "`" << std::endl;
                } else {
                    // find the right index for it in odrive_handles_
                    for (uint8_t i = 0; i < num_odrives_; ++i) {
                        if (0 == odrive_serial_numbers_[i].compare(std::to_string(read_serial_number))) { // found!  no need to close, but need to free device list
                            odrive_handles_[i] = device_handle;
                            attached_to_handle = true;
                            break;
                        }
                    }
                }
                if (!attached_to_handle) {
                    std::cerr << "Odrive with serial number " << std::to_string(read_serial_number) << " but not requested; ignoring odrive." << std::endl;
                    result = libusb_release_interface(device_handle, 0);
                    if (result != LIBUSB_SUCCESS) {
                        std::cerr << "Error calling libusb_release_interface on '0d" << std::to_string(desc.idVendor) << ":" << std::to_string(desc.idProduct) << "': `" << result << " - " << libusb_error_name(result) << "`" << std::endl;
                    }
                    libusb_close(device_handle);
                }
            }
        }
    }

    libusb_free_device_list(usb_device_list, 1);
    return ODRIVE_SDK_COMM_SUCCESS;
}


int ODriveDriver::odriveEndpointRequest(libusb_device_handle* handle, int endpoint_id, commBuffer& received_payload, int& received_length, commBuffer payload, int ack, int length) {
    commBuffer send_buffer;
    commBuffer receive_buffer;
    unsigned char receive_bytes[ODRIVE_SDK_MAX_RESULT_LENGTH] = { 0 };
    int sent_bytes = 0;
    int received_bytes = 0;
    short received_seq_no = 0;

    if (ack) {
        endpoint_id |= 0x8000;
    }
    outbound_seq_no_ = (outbound_seq_no_ + 1) & 0x7fff;
    outbound_seq_no_ |= 0x80; // FIXME, see odrive protocol.py
    short seq_no = outbound_seq_no_;

    // Send the packet
    commBuffer packet = createODrivePacket(seq_no, endpoint_id, length, payload);

    int result = libusb_bulk_transfer(handle, ODRIVE_SDK_WRITING_ENDPOINT, packet.data(), packet.size(), &sent_bytes, 0);
    if (result != LIBUSB_SUCCESS) {
        std::cerr << "Could not call libusb_bulk_transfer for writing: " << result << " - " << libusb_error_name(result) << strerror(errno) << std::endl;
        return result;
    } else if (packet.size() != sent_bytes) {
        std::cerr << "Could not call libusb_bulk_transfer: only wrote " << std::to_string(sent_bytes) << " of " << std::to_string(packet.size()) << " bytes (wanted to send `" << packet.data() << "`)" << std::endl;
    }

    if (ack) {
      // Immediatly wait for response from Odrive and check if ack (if we asked for one)
      result = libusb_bulk_transfer(handle, ODRIVE_SDK_READING_ENDPOINT, receive_bytes, ODRIVE_SDK_MAX_BYTES_TO_RECEIVE, &received_bytes, ODRIVE_SDK_TIMEOUT);
      if (result != LIBUSB_SUCCESS) {
          std::cerr << "Could not call libusb_bulk_transfer for reading: " << result << " - " << libusb_error_name(result) << strerror(errno) << std::endl;
          return result;
      }

      for (int i = 0; i < received_bytes; i++) {
          receive_buffer.push_back(receive_bytes[i]);
      }

      received_payload = decodeODrivePacket(receive_buffer, received_seq_no, receive_buffer);
      if (received_seq_no != seq_no) {
          std::cerr << "[ERROR] Recieved packet from odrive for sequence number " << std::to_string(received_seq_no) << " but was expecting to recieve reply for sequence number " << std::to_string(seq_no) << ".  Sequence ordering is not implemented yet!!!" << std::endl;
      }

      // return the response payload
      received_length = received_payload.size();
    }

    return LIBUSB_SUCCESS;
}

int ODriveDriver::odriveEndpointGetUInt8(libusb_device_handle* handle, int endpoint_id, uint8_t& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    int result = odriveEndpointRequest(handle, endpoint_id, receive_payload, received_length, send_payload, 1, 1);
    if (result != LIBUSB_SUCCESS) {
        return result;
    }

    deserializeCommBufferUInt8(receive_payload, value);

    return LIBUSB_SUCCESS;
}


int ODriveDriver::odriveEndpointGetFloat(libusb_device_handle* handle, int endpoint_id, float& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    int result = odriveEndpointRequest(handle, endpoint_id, receive_payload, received_length, send_payload, 1, 1);
    if (result != LIBUSB_SUCCESS) {
        return result;
    }
    
    std::cout << result <<  "Here: " << receive_payload.size() << std::endl;
    for ( int i = 0; i < receive_payload.size(); ++i) {
        std::cout << i << " "  << receive_payload[i] << std::endl;
    }
    deserializeCommBufferFloat(receive_payload, value);
    std::cout << value << std::endl;
    return LIBUSB_SUCCESS;
}


int ODriveDriver::odriveEndpointGetShort(libusb_device_handle* handle, int endpoint_id, short& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    int result = odriveEndpointRequest(handle, endpoint_id, receive_payload, received_length, send_payload, 1, 2);
    if (result != LIBUSB_SUCCESS) {
        return result;
    }

    readShortFromCommBuffer(receive_payload, value);

    return LIBUSB_SUCCESS;
}
int ODriveDriver::odriveEndpointGetInt(libusb_device_handle* handle, int endpoint_id, int& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    int result = odriveEndpointRequest(handle, endpoint_id, receive_payload, received_length, send_payload, 1, 4);
    if (result != LIBUSB_SUCCESS) {
        return result;
    }

    deserializeCommBufferInt(receive_payload, value);
    return LIBUSB_SUCCESS;
}

int ODriveDriver::odriveEndpointGetUInt64(libusb_device_handle* handle, int id, uint64_t& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    int result = odriveEndpointRequest(handle, id, receive_payload, received_length, send_payload, 1, 8);
    if (result != LIBUSB_SUCCESS) {
        return result;
    }
    deserializeCommBufferUInt64(receive_payload, value);
    return LIBUSB_SUCCESS;
}

int ODriveDriver::odriveEndpointSetInt(libusb_device_handle* handle, int endpoint_id, const int& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    serializeCommBufferInt(send_payload, value);
    int result = odriveEndpointRequest(handle, endpoint_id, receive_payload, received_length, send_payload, 1, 0);
    if (result != ODRIVE_SDK_COMM_SUCCESS) {
        return result;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int ODriveDriver::odriveEndpointSetFloat(libusb_device_handle* handle, int endpoint_id, const float& value) {
    commBuffer send_payload;
    commBuffer receive_payload;
    int received_length;
    serializeCommBufferFloat(send_payload, value);
    int result = odriveEndpointRequest(handle, endpoint_id, receive_payload, received_length, send_payload, 1, 0);
    if (result != ODRIVE_SDK_COMM_SUCCESS) {
        return result;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}


void ODriveDriver::appendShortToCommBuffer(commBuffer& buf, const short value) {
    buf.push_back((value >> 0) & 0xFF);
    buf.push_back((value >> 8) & 0xFF);
}


void ODriveDriver::readShortFromCommBuffer(commBuffer& byte_array, short& value) {
  //TODO: Check that -ve values are being converted correctly.
  value = 0;
  for(int i = 0; i < sizeof(short); ++i) {
     value <<= 8;
     value |= byte_array[i];
  }

  //Convert the byte array to little endian. It's currently being read in as a bigendian.
  value = be16toh(value);
}


void ODriveDriver::serializeCommBufferFloat(commBuffer& buf, const float& value) {
    for(int i = 0; i < sizeof(float); i++){
       buf.push_back(((unsigned char*)&value)[i]);
    }
}

void ODriveDriver::serializeCommBufferInt(commBuffer& buf, const int& value) {
    buf.push_back((value >> 0) & 0xFF);
    buf.push_back((value >> 8) & 0xFF);
    buf.push_back((value >> 16) & 0xFF);
    buf.push_back((value >> 24) & 0xFF);
}

void ODriveDriver::deserializeCommBufferFloat(commBuffer& byte_array, float& value) {
    for(int i =0; i < byte_array.size(); ++i) {
        std::cout << i << " " << byte_array[i] << " ";
    }
    std::cout << std::endl;
    memcpy(&value, &byte_array, byte_array.size());
}


void ODriveDriver::deserializeCommBufferInt(commBuffer& byte_array, int& value) {
  //TODO: Check that -ve values are being converted correctly.
  value = 0;
  for(int i = 0; i < byte_array.size(); ++i) {
     value <<= 8;
     value |= byte_array[i];
  }

  //Convert the byte array to little endian. It's currently being read in as a bigendian.
  value = be32toh(value);
}

void ODriveDriver::deserializeCommBufferUInt64(commBuffer& v, uint64_t& value) {
  value = 0;
  for(int i = 0; i < v.size(); ++i) {
     value <<= 8;
     value |= v[i];
  }

  //Convert the byte array to little endian. It's currently being read in as a bigendian.
  value = be64toh(value);
}

void ODriveDriver::deserializeCommBufferUInt8(commBuffer& v, uint8_t& value) {
  value = v[0];
}

commBuffer ODriveDriver::createODrivePacket(short seq_no, int endpoint_id, short response_size, const commBuffer& input) {
    commBuffer packet;
    short crc = 0;
    if ((endpoint_id & 0x7fff) == 0) {
        crc = ODRIVE_SDK_PROTOCOL_VERION;
    }
    else {
        crc = json_crc;
    }

    appendShortToCommBuffer(packet, seq_no);
    appendShortToCommBuffer(packet, endpoint_id);
    appendShortToCommBuffer(packet, response_size);

    for (uint8_t b : input) {
        packet.push_back(b);
    }

    appendShortToCommBuffer(packet, crc);

    return packet;
}

commBuffer ODriveDriver::decodeODrivePacket(commBuffer& buf, short& seq_no, commBuffer& received_packet) {
    commBuffer payload;
    readShortFromCommBuffer(buf, seq_no); // reads 2 bytes so start next for loop at 2
    seq_no &= 0x7fff;
    for (commBuffer::size_type i = 2; i < buf.size(); ++i) {
        payload.push_back(buf[i]);
    }
    return payload;
}
