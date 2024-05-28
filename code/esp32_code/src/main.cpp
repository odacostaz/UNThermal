#include "secrets.h"
#include "definitions.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <Adafruit_NeoPixel.h>



// Sensors and peripheral definitions
OneWire  oneWire(ONE_WIRE_BUS);   // connection to Dallas ds18b20 temperature sensor
DallasTemperature sensors(&oneWire);  // connection to Dallas ds18b20 temperature sensor
Adafruit_NeoPixel dispLeds(NUMPIXELS, LIGHT_PIN, NEO_GRB); // connection to rgb leds that display temperature


// Task handles for resuming and suspending  tasks
TaskHandle_t h_controlPidTask;
TaskHandle_t h_identifyTask;
TaskHandle_t h_generalControlTask;
TaskHandle_t h_publishStateTask;
TaskHandle_t h_menuTask;

// PID control default parameters
float kp = 16.796;
float ki = 2;
float kd  = 16.441;
float N = 27.38;
float beta = 0.1;
float h = SAMPLING_TIME; //sampling time
const float br = 1/0.99;
bool reset_int = false;


// Parameters of a general controller for the thermal system
unsigned int order;
float A [10][10];   // Controller's A matrix
float B [10][2];    // Controller's B matrix
float C [10];       // Controller's C matrix
float D [2];        // Controller's D matrix
float L [10];       // L is the gain matrix for the antiwindup observer


// loop control variables
float reference = DEFAULT_REFERENCE;   //  Desired temperature of reference
float y = 0;            //  Temperature
float u = 0;            //  Control signal
float usat = 0;         //  Control saturation signal


// Vectors of values and times for storing  the edges of
// an user's defined arbitrary signal
float stairs[50];       //  Arbitrary signal
uint32_t timeValues[50];


// Variables that keep the command sent by the user and the mode of control
unsigned int codeTopic = DEFAULT_TOPIC;        //Integer code of the command sent by the user
unsigned int typeControl = PID_CONTROLLER; //Integer code of mode of control,
// which can be PID or general control (default)


// These parameters allow configuring the different user's commands
float low_val =    0;
float high_val =   0;
uint32_t points_stairs;
uint32_t duration;
uint32_t points_high = 50;
uint32_t points_low = 50;
uint32_t np = 0;
uint32_t total_time = 4294967295;


uint16_t uee_points= (unsigned int) (20/h);
uint16_t divider = 35;
uint16_t prbs_points =  63 * divider;
uint16_t stab_points = (unsigned int) (150/h);


// Internet connection variables
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);


bool plusDetected = false;
bool minusDetected = false;
// These miscellaneous functions allow to receive formatted data (for instance controllers) from user
float hex2Float(const char* string){
    // This function converts a 4-digit hexadecimal string received from MQTT to a floating-point number.
    // It enables the reception of precise float numbers without rounding, minimizing the transmitted byte count.
    unsigned long temp;
    sscanf(string, "%8x", &temp);
    float f = *(float *)&temp;
    return f;
}

float hex2Long(const char* string){
    // This function converts a 4-digit hexadecimal string received from MQTT to a long integer.
    // As a result, it enables the reception of precise integer numbers with the minimum byte usage.
    long l = strtol(string, NULL, 16);
    return l;
}

char * float2Hex(float number){
    // This function converts a floating-point number to a 4-digit hexadecimal string. Consequently,
    // it enables the transmission of an exact float number with the minimum byte usage through MQTT.
    static char str[9];
    unsigned long ui;
    memcpy(&ui, &number, sizeof (ui));
    sprintf(str, "%08x", ui);
    return str;
}


char * long2Hex(unsigned long number){
    // This function converts a long integer number to a 4-digit hexadecimal string. Consequently,
    // it enables the transmission of a big number with the minimum byte usage through MQTT.
    static char str[9];
    unsigned long ui;
    memcpy(&ui, &number, sizeof (ui));
    sprintf(str, "%08x", ui);
    return str;
}

void hexStringToFloatArray(float* floatArray, const char* hexString, unsigned int numFloats) {
    // This function translates a user-sent hexadecimal string into an array of floats, enabling the transmission
    // of arbitrary signals and parameters of a controller in a state space form.
    for (size_t i = 0; i < numFloats; ++i) {
        unsigned long temp;
        sscanf(hexString + i * 8, "%8x", &temp); // Read 8 hex digits (32 bits)
        memcpy(floatArray + i, &temp, sizeof(float)); // Copy bits into float
    }

}

void hexStringToMatrix(float* matrix, const char* hexString, unsigned int order, unsigned int cols, unsigned int maxcols) {
    // This function translates a user-sent hexadecimal string into a matrix of floats, enabling the transmission
    // of arbitrary signals and parameters of a controller in a state space form.
    for (size_t i = 0; i < order; ++i) {
        unsigned long temp;
        for (size_t j = 0; j < cols; j++) {
            sscanf(hexString + (i*cols + j) * 8, "%8x", &temp); // Read 8 hex digits (32 bits)
            memcpy(matrix + (i*maxcols + j ), &temp, sizeof(float)); // Copy bits into float
        }
    }

}

void hexStringToLongArray(uint32_t *longArray, const char* hexString, unsigned int numLongs) {
    // This function translates a user-sent hexadecimal string into an array of floats, enabling the transmission
    // of arbitrary signals and parameters of a controller in a state space form.
    for (unsigned int  i = 0; i < numLongs; ++i) {
        uint32_t  temp;
        sscanf(hexString + i * 8, "%8x", &temp); // Read 8 hex digits (32 bits)
        memcpy(longArray + i, &temp, sizeof(uint32_t)); // Copy bits into float
    }
}

void sensorToColormap(float x, uint8_t * rgbval) {

    /** This function calculates the R, G, and B values of a fifth-degree polynomial approximation of the turbo colormap.
        It is the colormap displayed by the WS2812 LEDs connected to the thermal plant.
      The resulting colormap enables the user to visualize both the plant's temperature and the controller's actions.
      The input 'x' represents a normalized value within the [0, 1] range, corresponding to the analog variable visualized by the LEDs.
      The input 'rgbval' is a 3-element array designed to store the computed RGB values.
    */
    float r;
    float g;
    float b;

    // colormap turbo
    r = 0.1357 + x * (4.5974 - x * (42.3277 - x * (130.5887 - x * (150.5666 - x * 58.1375))));
    g = 0.0914 + x * (2.1856 + x * (4.8052 - x * (14.0195 - x * (4.2109 + x * 2.7747))));
    b = 0.1067 + x * (12.5925 - x * (60.1097 - x * (109.0745 - x * (88.5066 - x * 26.8183))));


    r = constrain(r,0,1);
    g = constrain(g,0,1);
    b = constrain(b,0,1);

    rgbval[0] = (uint8_t) 255*r;
    rgbval[1] = (uint8_t) 255*g;
    rgbval[2] = (uint8_t) 255*b;
    rgbval[0] = pgm_read_byte(&gamma8[rgbval[0]]);
    rgbval[1] = pgm_read_byte(&gamma8[rgbval[1]]);
    rgbval[2] = pgm_read_byte(&gamma8[rgbval[2]]);
}

void displayLed(float var, float valmin, float valmax, float start, uint8_t led){
    // This function takes an analog variable and visualizes it as a colormap using the "sensorToColormap" function.
    // The 'var' parameter represents the analog variable, while 'valmin', 'valmax', and 'percent' serve as adjustment
    // parameters to scale the colormap.
    // The 'led' parameter, with values 0 or 1, denotes each of the LEDs attached to the plant.
    uint8_t rgb[3];
    float x;
    x = start + constrain((1-start)* (var - valmin)/(valmax - valmin), 0, 1-start);
    sensorToColormap(x, rgb);
    dispLeds.setPixelColor(led, dispLeds.Color(rgb[0], rgb[1], rgb[2]));
    dispLeds.show();
}

float linearInterpolation(uint32_t t[], float r[], uint16_t n, uint32_t t_interp) {
    uint16_t i;
    float result;
    float r1;

    for (i = 0; i < n - 1; i++) {
        if (t[i] <= t_interp && t_interp <= t[i + 1]) {
            // Linear interpolation formula: y = y1 + (y2 - y1) * ((x - x1) / (x2 - x1))
            r1 = (float) ( t_interp -  t[i]) / (t[i + 1] - t[i]);
            result = r[i] + (r[i + 1] - r[i]) *  r1;
        }
    }
    return result;
}


void wattsToPlant(float watts){
    ledcWrite(PWM_CHANNEL, watts * percent2pwm);
}




void connectWiFi(){
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    printf("\n");
    printf("Connected to WIFI through IP: %s \n", WiFi.localIP().toString());

}



void connectMqtt()
{
    printf("Starting MQTT connection...");
    if (mqttClient.connect(THINGNAME, USER, PASSWORD))
    {
        mqttClient.subscribe(USER_SYS_STAIRS_CLOSED);
        mqttClient.subscribe(USER_SYS_SET_PID);
        mqttClient.subscribe(USER_SYS_SET_REF);
        mqttClient.subscribe(USER_SYS_STEP_CLOSED);
        mqttClient.subscribe(USER_SYS_STEP_OPEN);
        mqttClient.subscribe(USER_SYS_PRBS_OPEN);
        mqttClient.subscribe(USER_SYS_SET_GENCON);
        mqttClient.subscribe(USER_SYS_PROFILE_CLOSED);
        printf("connected to broker\n");
    }
    else
    {
        printf("Failed MQTT connection code %d \n try again in 3 seconds\n", mqttClient.state());
    }
}


// This function suspend all  controlling tasks
void suspendAllTasks(){
    vTaskSuspend(h_publishStateTask);
    vTaskSuspend(h_generalControlTask);
    vTaskSuspend(h_controlPidTask);
    vTaskSuspend(h_identifyTask);
    wattsToPlant(0);
}


void resumeControl(){
    reset_int = true;
    switch (typeControl) {
        case PID_CONTROLLER:
            vTaskResume(h_controlPidTask);
            break;
        case GENERAL_CONTROLLER_1P:
            vTaskResume(h_generalControlTask);
            break;
        case GENERAL_CONTROLLER_2P:
            vTaskResume(h_generalControlTask);
            break;
    }
}

// This function activate the default controller
void defaultControl(){
    codeTopic = DEFAULT_TOPIC;
    reset_int = true;
    vTaskSuspend(h_publishStateTask);
    vTaskSuspend(h_identifyTask);
    resumeControl();
}




void IRAM_ATTR onMqttReceived(char* lastTopic, byte* lastPayload, unsigned int lenght){
    suspendAllTasks();
    JsonDocument doc;
    if (strstr(lastTopic, USER_SYS_SET_PID)){
        typeControl = PID_CONTROLLER;
        deserializeJson(doc, lastPayload);
        kp = hex2Float((const char *) doc["kp"]);
        ki = hex2Float((const char *) doc["ki"]);
        kd = hex2Float((const char *) doc["kd"]);
        N = hex2Float((const char *) doc["N"]);
        beta = hex2Float((const char *) doc["beta"]);
        reset_int = true;
        printf("PID parameters settled:\n\tkp=%0.3f\n\tki=%0.3f\n\tkd=%0.3f\n\tN=%0.3f\n\tBeta=%0.3f\n",
               kp, ki, kd, N, beta);
        defaultControl();
    }

    else if (strstr(lastTopic, USER_SYS_SET_REF )){
        deserializeJson(doc, lastPayload);
        reference = hex2Float((const char *) doc["reference"]);
        reset_int = true;
        printf("Reference has been set to %0.2f degrees \n", reference);
        defaultControl();
    }

    else if (strstr(lastTopic, USER_SYS_STEP_CLOSED )){
        codeTopic = USER_SYS_STEP_CLOSED_INT;
        deserializeJson(doc, lastPayload);
        low_val = hex2Float((const char *) doc["low_val"]);
        high_val = hex2Float((const char *) doc["high_val"]);
        points_high = hex2Long((const char *) doc["points_high"]);
        points_low = hex2Long((const char *) doc["points_low"]);
        total_time = points_low + points_high;
        printf("Closed loop step response:\n");
        printf("    low value=%0.2f\n    high value=%0.2f\n    time in high =%0.2f\n    time in low=%0.2f\n", low_val,
               high_val, (float) (points_high * h), (float) (points_low * h));
        vTaskResume(h_publishStateTask);
        resumeControl();
    }

    else if (strstr(lastTopic, USER_SYS_STAIRS_CLOSED)){
        vTaskSuspend(h_identifyTask);
        vTaskSuspend(h_controlPidTask);
        vTaskSuspend(h_generalControlTask);
        codeTopic = USER_SYS_STAIRS_CLOSED_INT;
        deserializeJson(doc, lastPayload);
        points_stairs = hex2Long((const char *) doc["points"]);
        duration = hex2Long((const char *) doc["duration"]);
        const char *hexSignal = doc["signal"];
        hexStringToFloatArray(stairs, hexSignal, points_stairs);
        total_time = points_stairs * duration - 1;
        np = 0;
        Serial.printf("Stairs signal of %d steps  with a duration of %0.2f secs.\n", points_stairs, h * total_time);
        resumeControl();
    }

    else if(strstr(lastTopic, USER_SYS_PRBS_OPEN )) {
        codeTopic = USER_SYS_PRBS_OPEN_INT;
        deserializeJson(doc, lastPayload);
        high_val = hex2Float((const char *) doc["peak_amp"]);
        reference = hex2Float((const char *) doc["op_point"]);
        stab_points = hex2Long((const char *) doc["stab_points"]);
        uee_points = hex2Long((const char *) doc["uee_points"]);
        divider = hex2Long((const char *) doc["divider"]);
        prbs_points = 63 * divider;
        total_time = prbs_points + stab_points + uee_points;
        reset_int = true;
        np = 0;
        printf("Open loop test with a prbs signal with %d steps with a duration of %0.2f secs.\n",
               prbs_points, (total_time-1) * h);
        vTaskResume(h_publishStateTask);
        vTaskResume(h_identifyTask);
    }

    else if(strstr(lastTopic, USER_SYS_STEP_OPEN )){
        codeTopic = USER_SYS_STEP_OPEN_INT;
        vTaskSuspend(h_identifyTask);
        vTaskSuspend(h_controlPidTask);
        vTaskSuspend(h_generalControlTask);
        deserializeJson(doc, lastPayload);
        high_val = hex2Float((const char *) doc["amplitude"]);
        reference = hex2Float((const char *) doc["op_point"]);
        stab_points = hex2Long((const char *) doc["stab_points"]);
        uee_points = hex2Long((const char *) doc["uee_points"]);
        points_high = hex2Long((const char *) doc["points_high"]);
        total_time = points_high + stab_points + uee_points;
        reset_int = true;
        np = 0;
        printf("Open loop step response\n");
        printf("\tOperation point= %0.2f\n\tAmplitude=%0.2f\n\tDuration=%0.2f\n", reference, high_val, (points_high-1)*h);
        vTaskResume(h_publishStateTask);
        vTaskResume(h_identifyTask);
    }

    else if(strstr(lastTopic, USER_SYS_SET_GENCON)){
        deserializeJson(doc, lastPayload);
        order = hex2Long((const char *) doc["order"]);
        const char *hex_A = doc["A"];
        const char *hex_B = doc["B"];
        const char *hex_C = doc["C"];
        const char *hex_D = doc["D"];
        const char *hex_L = doc["L"];
        typeControl  = hex2Long((const char *) doc["typeControl"]);
        hexStringToMatrix( *A, hex_A, order, order, MAX_ORDER);
        hexStringToMatrix( *B, hex_B, order, 2, 2);
        hexStringToFloatArray(C, hex_C, order);
        hexStringToFloatArray(D, hex_D, 2);
        hexStringToFloatArray(L, hex_L, order);
        printf("A general controller or order %d has been loaded\n", order);
        printf("A=\n");
        for ( uint8_t  i =0; i < order; i++ ){
            for (size_t j =0; j < order; j++ ) {
                printf("%0.4f    ", A[i][j]);
            }
            printf("\n");
        }
        printf("B=\n");
        for (uint8_t  i =0; i < order; i++ ) {
            for (size_t j =0; j < 2; j++ ) {
                printf("%0.4f    ", B[i][j]);
            }
            printf("\n");
        }
        printf("C=\n");
        for (uint8_t  j =0; j < order; j++ ) {
            printf("%0.4f    ", C[j]);
        }
        printf("\n");
        printf("D = %0.4f   %0.4f \n", D[0], D[1]);
        printf("L=\n");
        for (uint8_t  j =0; j < order; j++ ) {
            printf("%0.4f    ", L[j]);
        }
        printf("\n");
        printf("control code: %d\n", typeControl);
        reset_int = true;
        defaultControl();
    }
    else if(strstr(lastTopic, USER_SYS_PROFILE_CLOSED)){
        codeTopic = USER_SYS_PROFILE_CLOSED_INT;
        deserializeJson(doc, lastPayload);
        points_stairs = hex2Long((const char *) doc["points"]);
        low_val = hex2Float((const char *) doc["min_val"]);
        high_val = hex2Float((const char *) doc["max_val"]);
        const char *timeValuesHex = doc["timevalues"];
        const char *stairsHex = doc["refvalues"];
        hexStringToFloatArray(stairs, stairsHex, points_stairs);
        hexStringToLongArray(timeValues, timeValuesHex, points_stairs);
        total_time = timeValues[points_stairs - 1];
        reset_int = true;
        printf("Closed loop profile response with a duration of %0.2f secs.\n", total_time * h);
        printf("   refvalues = [ ");
        for (uint8_t  j =0; j < points_stairs; j++ ) {
            printf("%0.2f ", stairs[j]);
        }
        printf("]\n");
        printf("   timevalues = [ ");
        for (uint8_t  j =0; j < points_stairs; j++ ) {
            printf("%0.2f ", timeValues[j] * h);
        }
        printf("]\n");
        resumeControl();
        vTaskResume(h_publishStateTask);

    }


}

void initMqtt() {
    mqttClient.setServer(BROKER, 1883);
    mqttClient.setCallback(onMqttReceived);
    mqttClient.setBufferSize (4096);
}

void publishStateClosed()
{
    JsonDocument doc;
    doc["np"] = long2Hex(np);
    doc["r"] = float2Hex(reference);
    doc["y"] = float2Hex(y);
    doc["u"] = float2Hex(usat);
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer); // print to client
    mqttClient.publish(SYS_USER_SIGNALS_CLOSED, jsonBuffer);
}

void publishStateOpen()
{
    JsonDocument doc;
    doc["np"] = long2Hex(np);
    doc["y"] = float2Hex(y);
    doc["u"] = float2Hex(usat);
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer); // print to client
    mqttClient.publish(SYS_USER_SIGNALS_OPEN, jsonBuffer);
}


float movingAverage(float newValue) {
    const int filterSize = 10;
    static float filter_values[filterSize] = {0.0};  // Array to store the last 'FILTER_SIZE' values
    static int index = 0;  // Current index in the array
    static float sum = 0.0;  // Sum of the current values in the array
    static int count = 0;  // Number of values added to the filter

    // Subtract the oldest value from sum and replace it with the new value
    sum = sum - filter_values[index] + newValue;
    filter_values[index] = newValue;

    // Update the index, and wrap around if necessary
    index = (index + 1) % filterSize ;

    // Keep track of how many values have been added to the filter
    if (count < filterSize ) {
        count++;
    }
    // Return the average of the values in the filter
    return sum / count;
}



void  computeReference() {
   switch (codeTopic) {
        case DEFAULT_TOPIC:
            displayLed(y, 0, 100, 0.25, 0);
            displayLed(reference, 0, 100, 0.25, 1);
            displayLed(usat, 0, 100, 0.2, 2);
            break;

        case USER_SYS_STEP_CLOSED_INT:
            if (np < points_low) {
                reference = low_val;
                //xTaskNotify(h_publishStateTask, 0b0001, eSetBits);
                displayLed(y, 10, 90, 0.3, 0);
                displayLed(reference, 10, 90, 0.3, 1);
                displayLed(usat, 0, 100, 0.1, 2);

            }
            else if (np <= total_time) {
                reference = high_val;
                //xTaskNotify(h_publishStateTask, 0b0001, eSetBits);
                displayLed(y, 10, 90, 0.3, 0);
                displayLed(reference, 10, 90, 0.3, 1);
                displayLed(usat, 0, 100, 0.1, 2);
            }
            else if (np <= total_time + 1){
                printf("Closed loop step response completed\n");
                reference = low_val;
                defaultControl();
            }
            break;

        case USER_SYS_STAIRS_CLOSED_INT:
            if (np <= total_time) {
                reference = stairs[np / duration];
                displayLed(reference, 20, 90,0.3, 0);
                displayLed(y, 20, 90, 0.3, 1);
            }
            else if (np == total_time + 1) {
                printf("Stairs closed loop response completed\n");
                ledcWrite(PWM_CHANNEL, 0 * percent2pwm);
                reference = DEFAULT_REFERENCE;
                defaultControl();
            }
            break;
       case USER_SYS_PROFILE_CLOSED_INT:
           if (np <= total_time) {
               reference = linearInterpolation(timeValues, stairs, points_stairs, np);
               displayLed(y, low_val, high_val, 0, 6);

           }
           else if (np == total_time + 1) {
               printf("Closed loop profile response completed\n");
               wattsToPlant(0);
               defaultControl();
           }
           break;
    }

}

float computeController(bool type){
    /* This function computes a general controlller for the thermal system
       The controller is defined by the state equation
           x[n+1] = (A - L * C) * x[k] + (B - L * D) * [r[n] y[n]]^T + L *u[n]
       where A, B, C ,D are the state equation matrices and L is the observer's gain for antiwindup,
       r[n] and y[n] are, respectively, the current reference and the current temperature
       of the thermal system

    */

    static float X[10] = {0};
    float Xnew[10] = {0};
    static float control = 0;
    float e;
    if (reset_int) {
        for (size_t  i = 0; i < order; i++){
            X[i] = 0;
        }
        control = 0;
        return 0;
    }
    else {
        e = reference - y;
        for (size_t i = 0; i < order; i++) {
            for (size_t j = 0; j < order; j++) {
                Xnew[i] += A[i][j] * X[j];
            }
            if (type) {
                Xnew[i] += B[i][0] * reference + B[i][1] * y + L[i] * control;
            }
            else{
                Xnew[i] += B[i][0] * e + L[i] * control;
            }
        }


        control = D[0] * reference + D[1] * y;
        for (size_t i = 0; i < order; i++) {
            control += C[i] * X[i];
        }
        for (size_t i = 0; i < order; i++) {
            X[i] = Xnew[i];
        }
        control = constrain(control, 0, 100);
        return control;
    }

}


static void controlPidTask(void *pvParameters) {
    /* this function computes a two parameter PID control with Antiwinduph
     See Astrom
    */
    const TickType_t taskPeriod = (pdMS_TO_TICKS(1000*h));
    float bi;
    float ad;
    float bd;
    float PA;              //  proportional action
    float DA;          //  derivative action
    float y_ant;       //  past output
    float IA;
    TickType_t xLastWakeTime;
    for (;;) {
        xLastWakeTime = xTaskGetTickCount();
        if (reset_int) {
            np = 0;
            IA = 0;
            reset_int = false;
            continue;
        }
        y = sensors.getTempCByIndex(0);
        computeReference();

        // This is a firmware protection in case of sensor failure or overtemperature.
        if (y <= 0 || y >= 100) {
            wattsToPlant(0);
            vTaskSuspend(nullptr);
        }
        bi = ki*h;
        ad = kd/(N*(kd/N + h));
        bd = kd/(kd/N + h);
        PA = kp * (beta * reference - y);
        DA = ad * DA - bd * (y - y_ant); // derivative action
        u = PA + IA + DA;// + uN ; // control signal
        usat =  constrain(u, 0, 100); //señal de control saturada
        wattsToPlant(usat); // Enviar señal de control en bits
        IA = IA + bi *(reference - y) + br*(usat - u);  // calculo de la accion integral
        y_ant = y;
        sensors.requestTemperatures();
        xTaskNotify(h_publishStateTask, 0b0001, eSetBits);
        xTaskDelayUntil(&xLastWakeTime, taskPeriod);
        np+=1;
    }
}



static void generalControlTask(void *pvParameters) {
    // Set the sampling time for the task at 750ms
    const TickType_t taskPeriod = (pdMS_TO_TICKS(1000*h));
    TickType_t xLastWakeTime;
    for (;;) {

        xLastWakeTime = xTaskGetTickCount();
        if (reset_int) {
            computeController(false);
            reset_int = false;
            np = 0;
            continue;
        }


        y = sensors.getTempCByIndex(0);
        computeReference();


        //This is to protect the system in case of sensor failure or overtemperature.

        if (y <= 0 || y >= 100) {
            wattsToPlant(0);
            vTaskSuspend(nullptr);
        }

        if (typeControl == GENERAL_CONTROLLER_1P){
            u = computeController(false);
        }
        else if (typeControl == GENERAL_CONTROLLER_2P) {
            u = computeController( true);
        }
        usat = constrain(u, 0, 100);
        wattsToPlant( usat);
        //printf("elapsed =%d\n", xTaskGetTickCount() - inicio);
        sensors.requestTemperatures();
        xTaskNotify(h_publishStateTask, 0b0001, eSetBits);
        xTaskDelayUntil(&xLastWakeTime, taskPeriod);
        np +=1;
    }
}




static void identifyTask(void *pvParameters) {
    /*
     This is the task for identification of the thermal plant
     by means of a PBRS signal
    */
    const TickType_t taskPeriod = (long) (1000 * h);
    const uint64_t pbrs = 0x57E08629E8E4B766;
    static uint bitShift = 0;
    float kp_id = 16.796;
    float beta_id = 0.5;
    float bi = 2 * h;
    bool  currBit;
    float PA;          //  proportional action
    float uf;          //  filtered control signal
    float IA;

    TickType_t xLastWakeTime;

    for (;;) {
        xLastWakeTime = xTaskGetTickCount();
        if (reset_int) {
            np = 0;
            IA = 0;
            reset_int = false;
            continue;
        }
        // This is for protecting the system both if the sensor fails or there is overtemperature
        if (y <= 0 || y >= 100) {
            ledcWrite(PWM_CHANNEL, 0 * percent2pwm);
            continue;
        }


        y = sensors.getTempCByIndex(0);

        if (np < stab_points) {
            PA = kp_id * (beta_id * reference - y);
            u = PA + IA ; // control signal
            usat =  constrain(u, 0, 100); //señal de control saturada
            wattsToPlant(usat);
            uf = movingAverage(usat);
            IA = IA + bi *(reference - y) + br*(usat - u);  // calculo de la accion integral
            displayLed(u, 0, 90, 0.25, 2);
            displayLed(y, 20, 90, 0.25, 0);
        }
        else if (np <= stab_points + uee_points) {
            usat = uf;
            wattsToPlant(usat);
            displayLed(usat, uf - 1.2*high_val, uf + 1.2*high_val, 0.25, 2);
            displayLed(y, reference - 5, reference + 5,  0.25,0);
        }

        else if ((np <= total_time) & (codeTopic == USER_SYS_PRBS_OPEN_INT)) {
            bitShift = (int) ((np - stab_points - uee_points) / divider);
            currBit = (pbrs >> bitShift) & 1;
            if (currBit) {
                usat = high_val + uf;
            } else {
                usat = uf - high_val;
            }
            wattsToPlant(usat);
        }
        else if ((np <= total_time) & (codeTopic == USER_SYS_STEP_OPEN_INT)) {
            usat = high_val + uf;
            wattsToPlant(usat);
        }
        else if (np == total_time + 1) {
            wattsToPlant(0);
            if (codeTopic == USER_SYS_STEP_OPEN_INT) {
                printf("Open loop step response completed\n");
            }
            if (codeTopic == USER_SYS_PRBS_OPEN_INT) {
                printf("PBRS open loop response completed\n");
            }
            defaultControl();
        }
        xTaskNotify(h_publishStateTask, 0b0010, eSetBits);
        sensors.requestTemperatures();
        vTaskDelayUntil(&xLastWakeTime, taskPeriod);
        np += 1;
    }
}


static void publishStateTask (void *pvParameters) {
    // local constants
    uint32_t rv;
    for (;;) {
        xTaskNotifyWait(0, 0b0011, &rv, portMAX_DELAY);
        if ( rv & 0b0001 ){
            publishStateClosed();
        }
        else if ( rv & 0b0010 ){
            publishStateOpen();
        }
    }
}



//void gotButtPlus() {
//    plusDetected = true;
//}
//
//void gotButtMinus() {
//    minusDetected = true;
//}
//
//
//
//static void menuTask(void *pvParameters) {
//    // Set the sampling time for the task at 750ms
//    for (;;) {
//        if (plusDetected) {
//            plusDetected = false;
//            if (touchInterruptGetLastStatus(BUTTON_PLUS)) {
//               reference = constrain(reference + 5,15,90);
//            }
//        }
//        if (minusDetected) {
//            minusDetected = false;
//            if (touchInterruptGetLastStatus(BUTTON_MINUS)) {
//                reference = constrain(reference - 5,15,90);
//            }
//        }
//       // printf("reference=%f       y=%f\n", reference, y);
//        vTaskDelay(1000);
//
//    }
//}


void handleConnections(void *pvParameters) {
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            connectWiFi();
        }
        if (!mqttClient.connected()) {
            vTaskDelay(3000);
            connectMqtt();
        }
        mqttClient.loop();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}






void setup() {
    // Initialize serial and wait for port to open:
    vTaskPrioritySet(nullptr,24);
    Serial.begin(115200);
    sensors.begin();

    sensors.requestTemperatures();
    pinMode(POWER_PIN , OUTPUT);
    ledcSetup(PWM_CHANNEL,  FREQUENCY_PWM, RESOLUTION_PWM);
    ledcAttachPin(POWER_PIN, PWM_CHANNEL);

    // Led Configuration
    dispLeds.begin();
    dispLeds.clear();
    dispLeds.setBrightness(20);
    dispLeds.show();

    xTaskCreatePinnedToCore(
            publishStateTask, // This is communication task
            "User command IOT",
            8192,
            NULL,
            12,
            &h_publishStateTask,
            CORE_COMM
    );

    // Create the task for PID control in core 0.
    xTaskCreatePinnedToCore(
            controlPidTask, // This is the control routine
            "PID control",
            8192,
            NULL,
            23,
            &h_controlPidTask,
            CORE_CONTROL
    );

   // vTaskSuspend(h_controlPidTask);

    // Create the task for general control in core 0.
    xTaskCreatePinnedToCore(
            generalControlTask, // This is the control routine
            "general control",
            8192,
            NULL,
            23,
            &h_generalControlTask,
            CORE_CONTROL
    );
    vTaskSuspend(h_generalControlTask);


    xTaskCreatePinnedToCore(
            identifyTask,
            "prbs-ident",
            8192,
            NULL,
            23,
            &h_identifyTask,
            CORE_CONTROL
    );
    vTaskSuspend(h_identifyTask);

//    /**  These functions activate the menu */
//    xTaskCreatePinnedToCore(
//            menuTask,
//            "prbs-ident",
//            2048,
//            NULL,
//            10,
//            &h_menuTask,
//            CORE_COMM
//    );
//
//
//    touchAttachInterrupt(BUTTON_PLUS, gotButtPlus, THRESHOLD_PLUS);
//    touchAttachInterrupt(BUTTON_MINUS, gotButtMinus, THRESHOLD_MINUS);


    /**  These functions activate communication */

    connectWiFi();
    initMqtt();
    connectMqtt();
    xTaskCreatePinnedToCore(
            handleConnections, // This is communication task
            "handle connections",
            4096,
            nullptr,
            1,
            nullptr,
            CORE_COMM // communications are attached to core 1
    );

}


void loop() {
    vTaskDelete(nullptr);
}


