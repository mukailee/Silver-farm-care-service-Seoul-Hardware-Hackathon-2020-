// ----------------------------------------------------------------------------
// Copyright 2016-2018 ARM Ltd.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------
#ifndef MBED_TEST_MODE

#include "mbed.h"
#include "simple-mbed-cloud-client.h"
#include "FATFileSystem.h"
#include "LittleFileSystem.h"

// Default network interface object. Don't forget to change the WiFi SSID/password in mbed_app.json if you're using WiFi.
NetworkInterface *net = NetworkInterface::get_default_instance();

// Default block device available on the target board
BlockDevice *bd = BlockDevice::get_default_instance();

#if COMPONENT_SD || COMPONENT_NUSD
// Use FATFileSystem for SD card type blockdevices
FATFileSystem fs("fs");
#else
// Use LittleFileSystem for non-SD block devices to enable wear leveling and other functions
LittleFileSystem fs("fs");
#endif

// Default User button for GET example and for resetting the storage
InterruptIn button(BUTTON1);
// Default LED to use for PUT/POST example
DigitalOut led(LED1, 0);

// How often to fetch sensor data (in seconds)
#define SENSORS_POLL_INTERVAL 3.0
#define MBED_CONF_IOTSHIELD_SENSOR_TEMP             A1
// Send all sensor data or just limited (useful for when running out of memory)
//#define SEND_ALL_SENSORS

// Sensors related includes and initialization
#include "FXOS8700Q.h"
#include "FXAS21002.h"

AnalogIn   tempVal(MBED_CONF_IOTSHIELD_SENSOR_TEMP); //19.08.06 added by tom

#ifdef TARGET_K66F
I2C sens_i2c(PTD9, PTD8);
#else
I2C sens_i2c(PTE25, PTE24);
#endif /* TARGET_K66F */

FXOS8700QAccelerometer sens_acc(sens_i2c, FXOS8700CQ_SLAVE_ADDR1);    // Configured for the FRDM-K64F with onboard sensors
FXOS8700QMagnetometer sens_mag(sens_i2c, FXOS8700CQ_SLAVE_ADDR1);
#ifdef TARGET_K66F
FXAS21002 sens_gyro(PTD9, PTD8, 0x20);
#endif /* TARGET_K66F */

// Declaring pointers for access to Pelion Device Management Client resources outside of main()
MbedCloudClientResource *res_button;
MbedCloudClientResource *res_led;
MbedCloudClientResource *res_post;
MbedCloudClientResource *res_temperature; //19.08.06 added by tom

// Additional resources for sensor readings
#ifdef SEND_ALL_SENSORS
MbedCloudClientResource *res_temperature; //19.08.06 added by tom
MbedCloudClientResource *res_magnometer_x;
MbedCloudClientResource *res_magnometer_y;
MbedCloudClientResource *res_magnometer_z;
MbedCloudClientResource *res_accelerometer_x;
MbedCloudClientResource *res_accelerometer_y;
MbedCloudClientResource *res_accelerometer_z;
#ifdef TARGET_K66F
MbedCloudClientResource *res_gyroscope_x;
MbedCloudClientResource *res_gyroscope_y;
MbedCloudClientResource *res_gyroscope_z;
#endif /* TARGET_K66F */
#endif /* SEND_ALL_SENSORS */

float temp =0;
float voltage = 0;    // 전압 계산 값 넣을 변수
float celsius = 0;    // 섭씨 계산 값 넣을 변수

void BG96_Modem_PowerON(void)
{
    DigitalOut BG96_RESET(D7);
    DigitalOut BG96_PWRKEY(D9);
 
    BG96_RESET = 1;
    BG96_PWRKEY = 1;
    wait_ms(200);
 
    BG96_RESET = 0;
    BG96_PWRKEY = 0;
    wait_ms(300);
 
    BG96_RESET = 1;
    wait_ms(5000);
}

// An event queue is a very useful structure to debounce information between contexts (e.g. ISR and normal threads)
// This is great because things such as network operations are illegal in ISR, so updating a resource in a button's fall() function is not allowed
EventQueue eventQueue;

// When the device is registered, this variable will be used to access various useful information, like device ID etc.
static const ConnectorClientEndpointInfo* endpointInfo;

/**
 * PUT handler - sets the value of the built-in LED
 * @param resource The resource that triggered the callback
 * @param newValue Updated value for the resource
 */
void put_callback(MbedCloudClientResource *resource, m2m::String newValue) {
    printf("*** PUT received, new value: %s                             \n", newValue.c_str());
    led = atoi(newValue.c_str());
}

/**
 * POST handler - prints the content of the payload
 * @param resource The resource that triggered the callback
 * @param buffer If a body was passed to the POST function, this contains the data.
 *               Note that the buffer is deallocated after leaving this function, so copy it if you need it longer.
 * @param size Size of the body
 */
void post_callback(MbedCloudClientResource *resource, const uint8_t *buffer, uint16_t size) {
    printf("*** POST received (length %u). Payload: ", size);
    for (size_t ix = 0; ix < size; ix++) {
        printf("%02x ", buffer[ix]);
    }
    printf("\n");
}

/**
 * Button handler
 * This function will be triggered either by a physical button press or by a ticker every 5 seconds (see below)
 */
void button_press() {
    int v = res_button->get_value_int() + 1;
    res_button->set_value(v);
    printf("*** Button clicked %d times                                 \n", v);
}

/**
 * Notification callback handler
 * @param resource The resource that triggered the callback
 * @param status The delivery status of the notification
 */
void button_callback(MbedCloudClientResource *resource, const NoticationDeliveryStatus status) {
    printf("*** Button notification, status %s (%d)                     \n", MbedCloudClientResource::delivery_status_to_string(status), status);
}

/**
 * Registration callback handler
 * @param endpoint Information about the registered endpoint such as the name (so you can find it back in portal)
 */
void registered(const ConnectorClientEndpointInfo *endpoint) {
    printf("Registered to Pelion Device Management. Endpoint Name: %s\n", endpoint->internal_endpoint_name.c_str());
    endpointInfo = endpoint;
}

/**
 * Initialize sensors
 */
void sensors_init() {
    printf ("\nSensors configuration:\n");

    sens_acc.enable();
    sens_mag.enable();
#ifdef TARGET_K66F
    sens_gyro.activate(true);
#endif /* TARGET_K66F */

    printf("FXOS8700Q accelerometer = 0x%X\n", sens_acc.whoAmI());
    printf("FXOS8700Q magnetometer  = 0x%X\n", sens_mag.whoAmI());
#ifdef TARGET_K66F
    printf("FXAS21002 gyroscope     = 0x%X\n", sens_gyro.getStatus());
#endif /* TARGET_K66F */

    printf("\n"); ;
}

float getTemperature_C(float _voltage)
{
    // LM35는 섭씨 1도당 10mV의 전위차를 갖는다.
    // 센서핀의 전압이 0.28V라면 280mV이므로 온도는 28도씨.
    // 100을 곱해서 섭씨 온도로 나타냄
    celsius = voltage * 100.0;
    return celsius;
}


/**
 * Update sensors and report their values.
 * This function is called periodically.
 */
void sensors_update() {

    motion_data_counts_t acc_raw, mag_raw;

    sens_acc.getAxis(acc_raw);
    sens_mag.getAxis(mag_raw);

    float mag_x = (double)mag_raw.x / 1000.0, mag_y = (double)mag_raw.y / 1000.0, mag_z = (double)mag_raw.z / 1000.0;
    float acc_x = (double)acc_raw.x / 1000.0, acc_y = (double)acc_raw.y / 1000.0, acc_z = (double)acc_raw.z / 1000.0;
    
    temp = tempVal.read_u16()/100;
    voltage = temp * 4.95 / 1024;
    celsius = getTemperature_C(voltage);
    printf("Celsius temp : %.2f C", celsius);
    if (endpointInfo) {
        res_temperature->set_value(celsius);
    }
    
#ifdef TARGET_K66F
    float gyro_x = (double)sens_gyro.getX() / 1000.0, gyro_y = (double)sens_gyro.getY() / 1000.0, gyro_z = (double)sens_gyro.getZ() / 1000.0;
#endif /* TARGET_K66F */

    printf("                                                             \n");
    printf("FXOS8700Q mag:  %7.3f x, %7.3f y, %7.3f z [gauss]        \n", mag_x, mag_y, mag_z);
    printf("FXOS8700Q acc:  %7.3f x, %7.3f y, %7.3f z [g]            \n", acc_x, acc_y, acc_z);
#ifdef TARGET_K66F
    printf("FXAS21002 gryo: %7.3f x, %7.3f y, %7.3f z [dps]          \n", gyro_x, gyro_y, gyro_z);
    printf("\r\033[4A");
#else
    printf("\r\033[3A");
#endif /* TARGET_K66F */

    if (endpointInfo) {
#ifdef SEND_ALL_SENSORS
        res_temperature->set_value(celsius);
        res_accelerometer_x->set_value(acc_x);
        res_accelerometer_y->set_value(acc_y);
        res_accelerometer_z->set_value(acc_z);
        res_magnometer_x->set_value(mag_x);
        res_magnometer_y->set_value(mag_y);
        res_magnometer_z->set_value(mag_z);
#ifdef TARGET_K66F
        res_gyroscope_x->set_value(gyro_x);
        res_gyroscope_y->set_value(gyro_y);
        res_gyroscope_z->set_value(gyro_z);
#endif /* TARGET_K66F */
#endif /* SEND_ALL_SENSORS */
    }
}




int main(void) {
    printf("\nStarting Simple Pelion Device Management Client example\n");

    int storage_status = fs.mount(bd);
    if (storage_status != 0) {
        printf("Storage mounting failed.\n");
    }
#if USE_BUTTON == 1
    // If the User button is pressed ons start, then format storage.
    bool btn_pressed = (button.read() == MBED_CONF_APP_BUTTON_PRESSED_STATE);
    if (btn_pressed) {
        printf("User button is pushed on start...\n");
    }
#else
    bool btn_pressed = FALSE;
#endif /* USE_BUTTON */

    if (storage_status || btn_pressed) {
        printf("Formatting the storage...\n");
        int storage_status = StorageHelper::format(&fs, bd);
        if (storage_status != 0) {
            printf("ERROR: Failed to reformat the storage (%d).\n", storage_status);
        }
    } else {
        printf("You can hold the user button during boot to format the storage and change the device identity.\n");
    }

    BG96_Modem_PowerON();
    printf("M2Mnet(BG96) Power ON\n");
    
    sensors_init();

    // Connect to the Internet (DHCP is expected to be on)
    printf("Connecting to the network using the default network interface...\n");
    net = NetworkInterface::get_default_instance();

    nsapi_error_t net_status = NSAPI_ERROR_NO_CONNECTION;
    while ((net_status = net->connect()) != NSAPI_ERROR_OK) {
        printf("Unable to connect to network (%d). Retrying...\n", net_status);
    }

    printf("Connected to the network successfully. IP address: %s\n", net->get_ip_address());

    printf("Initializing Pelion Device Management Client...\n");

    // SimpleMbedCloudClient handles registering over LwM2M to Pelion Device Management
    SimpleMbedCloudClient client(net, bd, &fs);
    int client_status = client.init();
    if (client_status != 0) {
        printf("Pelion Client initialization failed (%d)\n", client_status);
        return -1;
    }

    // Creating resources, which can be written or read from the cloud
    
    res_temperature = client.create_resource("3303/0/5700", "Temperature (C)");
    res_temperature->set_value(0);
    res_temperature->methods(M2MMethod::GET);
    res_temperature->observable(true);
    
    res_button = client.create_resource("3200/0/5501", "button_count");
    res_button->set_value(0);
    res_button->methods(M2MMethod::GET);
    res_button->observable(true);
    res_button->attach_notification_callback(button_callback);

    res_led = client.create_resource("3201/0/5853", "led_state");
    res_led->set_value(led.read());
    res_led->methods(M2MMethod::GET | M2MMethod::PUT);
    res_led->attach_put_callback(put_callback);

    res_post = client.create_resource("3300/0/5605", "execute_function");
    res_post->methods(M2MMethod::POST);
    res_post->attach_post_callback(post_callback);

#ifdef SEND_ALL_SENSORS
    res_accelerometer_x = client.create_resource("3313/0/5702", "Accelerometer X");
    res_accelerometer_x->set_value(0);
    res_accelerometer_x->methods(M2MMethod::GET);
    res_accelerometer_x->observable(true);

    res_accelerometer_y = client.create_resource("3313/0/5703", "Accelerometer Y");
    res_accelerometer_y->set_value(0);
    res_accelerometer_y->methods(M2MMethod::GET);
    res_accelerometer_y->observable(true);

    res_accelerometer_z = client.create_resource("3313/0/5704", "Accelerometer Z");
    res_accelerometer_z->set_value(0);
    res_accelerometer_z->methods(M2MMethod::GET);
    res_accelerometer_z->observable(true);

    res_magnometer_x = client.create_resource("3314/0/5702", "Magnometer X");
    res_magnometer_x->set_value(0);
    res_magnometer_x->methods(M2MMethod::GET);
    res_magnometer_x->observable(true);

    res_magnometer_y = client.create_resource("3314/0/5703", "Magnometer Y");
    res_magnometer_y->set_value(0);
    res_magnometer_y->methods(M2MMethod::GET);
    res_magnometer_y->observable(true);

    res_magnometer_z = client.create_resource("3314/0/5704", "Magnometer Z");
    res_magnometer_z->set_value(0);
    res_magnometer_z->methods(M2MMethod::GET);
    res_magnometer_z->observable(true);

#ifdef TARGET_K66F
    res_gyroscope_x = client.create_resource("3334/0/5702", "Gyroscope X");
    res_gyroscope_x->set_value(0);
    res_gyroscope_x->methods(M2MMethod::GET);
    res_gyroscope_x->observable(true);

    res_gyroscope_y = client.create_resource("3334/0/5703", "Gyroscope Y");
    res_gyroscope_y->set_value(0);
    res_gyroscope_y->methods(M2MMethod::GET);
    res_gyroscope_y->observable(true);

    res_gyroscope_z = client.create_resource("3334/0/5704", "Gyroscope Z");
    res_gyroscope_z->set_value(0);
    res_gyroscope_z->methods(M2MMethod::GET);
    res_gyroscope_z->observable(true);
#endif /* TARGET_K66F */
#endif /* SEND_ALL_SENSORS */

    printf("Initialized Pelion Device Management Client. Registering...\n");

    // Callback that fires when registering is complete
    client.on_registered(&registered);

    // Register with Pelion DM
    client.register_and_connect();

    // The button fires on an interrupt context, but debounces it to the eventqueue, so it's safe to do network operations
    button.fall(eventQueue.event(&button_press));
    printf("Press the user button to increment the LwM2M resource value...\n");

    // The timer fires on an interrupt context, but debounces it to the eventqueue, so it's safe to do network operations
    Ticker timer;
    timer.attach(eventQueue.event(&sensors_update), SENSORS_POLL_INTERVAL);

    // You can easily run the eventQueue in a separate thread if required
    eventQueue.dispatch_forever();
}


#endif /* MBED_TEST_MODE */
