#include "mbed.h"
#include "mbed_rpc.h"
#include <cmath>

#include "accelerometer_handler.h"
#include "config.h"
#include "magic_wand_model_data.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

#include "accelerometer_handler.h"
#include "stm32l475e_iot01_accelero.h"

// GLOBAL VARIABLES
WiFiInterface *wifi;
//InterruptIn btn3(SW3);
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;

const char* topic = "Mbed";

RpcDigitalOut myled1(LED1,"myled1");
RpcDigitalOut myled2(LED2,"myled2");
RpcDigitalOut myled3(LED3,"myled3");
InterruptIn button(USER_BUTTON);
BufferedSerial pc(USBTX, USBRX);
void gc(Arguments *in, Reply *out);
RPCFunction rpcLED(&gc, "gc");
void tilt(Arguments *in, Reply *out);
RPCFunction rpcLED2(&tilt, "tilt");
int degree = 30;
int final_degree = 30;
float over_degrees[10];

EventQueue queue(32 * EVENTS_EVENT_SIZE);

int16_t DataXYZ[3] = {0};
int16_t G_pDataXYZ[3] = {0};

Thread t;

int GFlag = 1;
int TFlag = 1;

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
    message_num++;
    MQTT::Message message;
    char buff[100];
    sprintf(buff, "QoS0 Hello, Python! #%d", message_num);
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client->publish(topic, message);

    printf("rc:  %d\r\n", rc);
    printf("Puslish message: %s\r\n", buff);
}

void close_mqtt() {
    closed = true;
}

void stopGMode(){
  GFlag = 0;
  //printf("final : %d\n", degree);
  //printf("Gflag = %d\n", GFlag);
  final_degree = degree;
  printf("final = %d\n", final_degree);
}

int main() {
    //The mbed RPC classes are now wrapped to create an RPC enabled version - see RpcClasses.h so don't add to base class
    t.start(callback(&queue, &EventQueue::dispatch_forever));

    GFlag = 1;

    BSP_ACCELERO_Init();

    button.rise(queue.event(stopGMode));

    // receive commands, and send back the responses
    char buf[256], outbuf[256];

    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");

    wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
    }


    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
    }


    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

    //TODO: revise host to your IP
    const char* host = "192.168.111.169";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return -1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }

    while(1) {
        memset(buf, 0, 256);
        for (int i = 0; ; i++) {
            char recv = fgetc(devin);
            if (recv == '\n') {
                printf("\r\n");
                break;
            }
            buf[i] = fputc(recv, devout);
        }
        //Call the static call method on the RPC class
        RPC::call(buf, outbuf);
        printf("%s\r\n", outbuf);
    }
}

constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

// Return the result of the last prediction
int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}

int gesture_main() {
  
  // Whether we should clear the buffer next time we fetch data
  bool should_clear_buffer = false;
  bool got_data = false;

  // The gesture index of the prediction
  int gesture_index;

  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return -1;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != config.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return -1;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return -1;
  }

  error_reporter->Report("Set up successful...\n");

  while (GFlag) {
    //printf("flag = %d\n", GFlag);

    // Attempt to read new data from the accelerometer
    got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);

    // If there was no new data,
    // don't try to clear the buffer again and wait until next time
    if (!got_data) {
      should_clear_buffer = false;
      continue;
    }

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      error_reporter->Report("Invoke failed on index: %d\n", begin_index);
      continue;
    }

    // Analyze the results to obtain a prediction
    gesture_index = PredictGesture(interpreter->output(0)->data.f);

    // Clear the buffer next time we read data
    should_clear_buffer = gesture_index < label_num;

    // Produce an output
    if (gesture_index < label_num) {
      //error_reporter->Report(config.output_message[gesture_index]);
      //printf("%d\n", gesture_index);
      if (gesture_index == 1 && degree < 50){
        degree += 5;
        printf("%d\n", degree);
      } else if (gesture_index == 0 && degree > 30){
        degree -= 5;
        printf("%d\n", degree);
      }
    }
    //GFlag = 0;
    //printf("flag = %d\n", GFlag);
  }
}

// Make sure the method takes in Arguments and Reply objects.
void gc (Arguments *in, Reply *out)   {
    myled1.write(1);
    ThisThread::sleep_for(500ms);
    myled1.write(0);
    ThisThread::sleep_for(500ms);
    myled1.write(1);
    ThisThread::sleep_for(500ms);
    myled1.write(0);
    ThisThread::sleep_for(500ms);
    myled1.write(1);
    ThisThread::sleep_for(500ms);
    myled1.write(0);
    ThisThread::sleep_for(500ms);
    gesture_main();
}

void tilt (Arguments *in, Reply *out)   {
    myled2.write(1);
    ThisThread::sleep_for(500ms);
    myled2.write(0);
    ThisThread::sleep_for(500ms);
    myled2.write(1);
    ThisThread::sleep_for(500ms);
    myled2.write(0);
    ThisThread::sleep_for(500ms);
    myled2.write(1);
    ThisThread::sleep_for(500ms);
    myled2.write(0);
    ThisThread::sleep_for(500ms);
    BSP_ACCELERO_AccGetXYZ(G_pDataXYZ);
    int cnt = 0;
    TFlag = 1;
    while (TFlag)
    {
      
      BSP_ACCELERO_AccGetXYZ(DataXYZ);
      //printf("%lf, %lf, %lf\n", DataXYZ[0], DataXYZ[1], DataXYZ[2]);

      double a = sqrt(pow(G_pDataXYZ[0], 2) + pow(G_pDataXYZ[1], 2) + pow(G_pDataXYZ[2], 2));
      double b = sqrt(pow(DataXYZ[0], 2) + pow(DataXYZ[1], 2) + pow(DataXYZ[2], 2));
      double c = sqrt(pow((DataXYZ[0] - G_pDataXYZ[0]),2) + pow((DataXYZ[1] - G_pDataXYZ[1]),2) + pow((DataXYZ[2] - G_pDataXYZ[2]),2));
      ///printf("%lf, %lf, %lf\n", a, b, c);
      double cos = (a*a+b*b-c*c) / (2*a*b);
      double theta = acos((pow(a,2) + pow(b,2) - pow(c,2)) / (2 * a * b)) * 180 / 3.1415926;

      if (theta > final_degree){
        over_degrees[cnt] = theta;
        cnt++;
        printf("over degree : %lf\n", theta);
      } else {
        //printf("not over : %lf\n", theta);
        //printf("cos = %lf\n", cos);
      }

      if (cnt >= 10){
        TFlag = 0;
      }
      ThisThread::sleep_for(100ms);
    }
    
}