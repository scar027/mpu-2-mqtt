# mpu-2-mqtt
ESP-IDF implementation of reading and publishing sensor data via 2 MPU6050s connected to an ESP32 over MQTT TLS(MQTTS).
- Built referring [Espressif MQTT SSL implementation example](https://github.com/espressif/esp-idf/tree/v5.4.1/examples/protocols/mqtt/ssl) and [Espressif MPU6050 sensor implementation example](https://github.com/espressif/esp-bsp/blob/master/examples/display_sensors/main/sensors_example.c).
- Modifies the original [MPU6050 component](https://components.espressif.com/components/espressif/mpu6050/versions/1.2.0) present in ESP component registry to provide a library to support a dual MPU6050 configuration.
- Integrates the newer "driver/i2c_master.h" driver in the original MPU6050 component source code.
- Provides easy Broker Configuration and WiFi SSID and Password configuration via menuconfig.

## Get MQTT Certificates for TLS
1. Navigate to the main/cert/ folder and run the following commands(requires openssl installed):
    ```bash
    cd main/cert/
    # Generate private key(client key)
    openssl genrsa -out client.key
    # Generate client certificate signing request(client.csr) based on the private key
    # Enter the country, organization, and name when prompted. Leave fields blank by typing a dot (.) if you don’t want to fill them.
    openssl req -out client.csr -key client.key -new
    ```
1. Now generate the client certificate(client.crt) at <https://test.mosquitto.org/ssl/> by pasting the contents of client.csr there. You can get the contents from the terminal itself using:
    ```bash
    more client.csr
    ```
1. Store the generated client.crt file in the main/cert/ folder as well.
1. Fetch the mosquitto.org CA(Certificate Authority) certificate (mosquitto.org.pem). You can also do this directly from the website as well and then storing the downloaded file in the main/cert/ folder.
    ```bash
    curl -o mosquitto.org.crt https://test.mosquitto.org/ssl/mosquitto.org.crt
    ```
1. Install Eclipse Mosquitto on your desktop from <https://mosquitto.org/download/>
- Tip: For Ubuntu use the ppa instead of snap. Here's [why](https://stackoverflow.com/questions/67262895/mosquitto-sub-fails-with-error-problem-setting-tls-options-file-not-found-w).
1. Test by subscribing to a secure server on a different terminal(in the same folder):
    ```bash
    mosquitto_sub -h test.mosquitto.org -t "testcert" -p 8884 --cafile mosquitto.org.crt --key client.key --cert client.crt -d
    ```
1. Publish to the topic to test:
    ```bash
    mosquitto_pub --cafile mosquitto.org.crt --key client.key --cert client.crt -h test.mosquitto.org -m "Hello World!" -t "testcert" -p 8884 -d
    ```
1. Once you have verified that it works create a new file with the name mqtt_eclipseprojects_io.pem and copy the contents of the client.key, the client.crt and the mosquitto.org.crt in it(in the same order). You can do this by running this command from the main/ directory:
    ```bash
    cat cert/client.key cert/client.crt cert/mosquitto.org.crt > mqtt_eclipseprojects_io.pem
    ```

## Set Target
1. After cloning the repository, set the intended target. For an ESP32 this can be done:
```bash
idf.py set-target esp32
```

## Build the project
1. This will help add the additionally defined options for Broker configuration and WiFi configuration in the default menuconfig.
```bash
idf.py build
```

## Configure MQTT Broker and WiFi SSID and Password
```bash
idf.py menuconfig
```

1. Navigate to Broker Configuration and change the Broker URL if required. Use enter to enter and esc to exit. 
1. Navigate to WiFi Configuration and enter the SSID and Password for your network.
1. Enter 'S' to save the configuration.
1. Rebuild:
```bash
idf.py build
```

## Flash and Monitor
1. Connect the ESP32 via USB and upload the code and monitor using:
```bash
idf.py flash monitor
```

## Verify by receiving
1. Open a new terminal window and check that data is being received by subscribing to the MQTTS topic. Make sure you are in the directory containing mosquitto.org.crt (main/cert/ in this repository)before running this.
```bash
mosquitto_sub -h test.mosquitto.org -p 8883 --cafile mosquitto.org.crt -t "sensors1635/mpu6050"
```