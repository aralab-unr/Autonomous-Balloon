#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>
#include <SoftwareSerial.h>

#define RX_PIN D7  
#define TX_PIN D8 
SoftwareSerial opticalflowserial(RX_PIN, TX_PIN);

const char* ssid = "NETGEAR80";
const char* password = "grandoboe861";

const char* host = "192.168.1.9"; 
const uint16_t port = 1234;

WiFiClient client;

unsigned long lastSendTime = 0;
int mode = 0;
int direction = 0;
int velocity = 0;

struct State {
  double x, y;
  int z;
};

double epsilon = 0.95;
double epsilon_decay = 0.875;
int episode = 0;
State state = {0, 0, 1};
State prestate = {0, 0, 1};

double belief[5][4];
double prev_belief[5][4];

int actions[3] = {1, -1, 0};
int areaSize[3] = {100, 100, 5};
int goalState[2] = {0, 0};

int hdes = 1.0;

double zr = 0.09;
double arrival_time = -1;
bool is_at_hdes = false;
int actionIdx;
double time_now = 0.0;

const int possibleWinds[4][2] = {
  {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
};

uint32_t LoopTimer;
float dt = 0.01;

float x = 0, y = 0, z = 0;
float xd = 0, yd = 0, zd = 0;

const float Ratm = 287.0;
const float Kssmc = 0.075;
const float Ksmc = 0.125;
const float m = 0.080;
const float g = 9.80;
const float RHe = 2077.0;
const float mhe = 0.0085;
const  float The = 298.0;
const float Tatm = 298.0;
const float madd = 0.04;

const float lambda = 2.25;
const float gamma_val = 0.075;
const float epsat = 0.1;

float fz = 0.0;
float dotDz = 0.0;
float Dz = 0.0;

float dtOptical = 0.02;

const int IN1 = D1;  
const int IN2 = D2;  
const int IN3 = D5; 
const int IN4 = D6; 
int pw1 = 0, pw2 = 0, pw3 = 0, pw4 = 0; 

#define MICOLINK_MSG_HEAD 0xEF
#define MICOLINK_MAX_PAYLOAD_LEN 64
#define MICOLINK_MAX_LEN (MICOLINK_MAX_PAYLOAD_LEN + 7)

enum {
    MICOLINK_MSG_ID_RANGE_SENSOR = 0x51,
};

typedef struct {
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum;

    // internal parsing state
    uint8_t status = 0;
    uint8_t payload_cnt = 0;
} MICOLINK_MSG_t;

#pragma pack(push, 1)
typedef struct {
    uint32_t time_ms;
    uint32_t distance;
    uint8_t strength;
    uint8_t precision;
    uint8_t dis_status;
    uint8_t reserved1;
    int16_t flow_vel_x;
    int16_t flow_vel_y;
    uint8_t flow_quality;
    uint8_t flow_status;
    uint16_t reserved2;
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack(pop)

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 4; j++) {
      belief[i][j] = 1.0 / 4.0;
      prev_belief[i][j] = 1.0 / 4.0;
    }
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("ESP8266 IP: ");
  Serial.println(WiFi.localIP());

  opticalflowserial.begin(115200);

  while (!client.connect(host, port)) {
    Serial.println("Connecting to server...");
    delay(1000);
  }
  Serial.println("Connected to server");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  LoopTimer = micros();
}

void loop() {

  getoptical();
  if (millis() - lastSendTime >= 200) {  
    lastSendTime = millis();
    String payload = "x =" + String(x, 2) + ", y =" + String(y, 2) + ", z =" + String(z, 2) + ", fz =" + String(fz, 2) + ", Dz =" + String(Dz, 2) + ", zr =" + String(zr, 2) + ", actionIdx =" + String(actionIdx) + ", epsilon =" + String(epsilon, 3);
    payload += ", belief=";
    for (int i = 0; i < 5; ++i) {
      for (int j = 0; j < 4; ++j) {
        payload += String(belief[i][j], 2);
        if (i != 4 || j != 3) payload += ","; 
      }
    }
    payload += "\n";

    client.print(payload);

    while (client.available()) {
      String cmd = client.readStringUntil('\n');
      cmd.trim();  
      Serial.print("Received command: ");
      Serial.println(cmd);

      // Split the input string into 3 parts: mode, direction, velocity
      int firstComma = cmd.indexOf(',');
      int secondComma = cmd.indexOf(',', firstComma + 1);

      if (firstComma > 0 && secondComma > firstComma) {
        String modeStr = cmd.substring(0, firstComma);
        String directionStr = cmd.substring(firstComma + 1, secondComma);
        String velocityStr = cmd.substring(secondComma + 1);

        int equalIndexMode = modeStr.indexOf('=');
        int equalIndexDir = directionStr.indexOf('=');
        int equalIndexVel = velocityStr.indexOf('=');

        if (equalIndexMode > 0 && equalIndexDir > 0 && equalIndexVel > 0) {
          mode = modeStr.substring(equalIndexMode + 1).toInt();
          direction = directionStr.substring(equalIndexDir + 1).toInt();
          velocity = velocityStr.substring(equalIndexVel + 1).toInt();
        } else {
          Serial.println("Error: '=' not found in one of the fields.");
        }
      } else {
        Serial.println("Invalid format. Expected: mode=INT,direction=INT,velocity=INT");
      }
    }
  }
  if (mode == 0 & direction == 0) {
    Dz = 0;
    pw1 = velocity;
    pw2 = 0;
    pw3 = 0;
    pw4 = velocity;
  }
  else if (mode == 0 & direction == 1) {
    Dz = 0;
    pw1 = 0;
    pw2 = velocity;
    pw3 = velocity;
    pw4 = 0;
  }
  else if (mode == 1) {
    time_now = millis() / 1000.0;
    double h = z;
    double vh = zd;
    int current_z = constrain((int)(h / 0.4), 1, areaSize[2]);
    double current_pos[3] = {
      constrain(x, -areaSize[0], areaSize[0]),
      constrain(y, -areaSize[1], areaSize[1]),
      (double)current_z
    };
    if (current_z == hdes) {
      if (!is_at_hdes) {
        arrival_time = time_now;
        is_at_hdes = true;
      }
      if ((time_now - arrival_time) >= 1 && arrival_time > 0) {
        episode++;
        state.x = current_pos[0];
        state.y = current_pos[1];
        state.z = current_pos[2];

        updateBelief(prestate.x, prestate.y, state.x, state.y, prestate.z);
        epsilon = max(epsilon * epsilon_decay, 0.001);

        actionIdx = chooseAction(state, belief);
        hdes = state.z + actions[actionIdx];
        hdes = constrain(hdes, 1, areaSize[2]);

        double max_belief_change = 0;
        for (int i = 0; i < 5; i++) {
          for (int j = 0; j < 4; j++) {
            double diff = fabs(belief[i][j] - prev_belief[i][j]);
            if (diff > max_belief_change) max_belief_change = diff;
          }
        }
        if (max_belief_change > 0.25) {
          epsilon = 0.975;
          Serial.print("Belief update threshold crossed: ");
          Serial.println(max_belief_change, 2);
        }

        arrival_time = -1;
        is_at_hdes = false;
        memcpy(prev_belief, belief, sizeof(belief));
        prestate = state;
      }
    } else {
      arrival_time = -1;
      is_at_hdes = false;
    }
    zr = hdes * 0.4 + 0.15;
    //zr = 0.6;
    float zrd = 0.3 * (zr - z);
    float s = (zd - zrd) + lambda * (z - zr);
    float sat_s;
    if (abs(s) > epsat) {
      sat_s = (s > 0) ? 1.0 : -1.0;
    } else {
      sat_s = s / epsat;
    }
    dotDz = gamma_val * s;
    Dz = Dz + dotDz * dt;
    Dz = constrain(Dz, -0.15, 0.15);
    fz = (-Kssmc * sat_s - Ksmc * s - lambda * (zd - zrd)) * (m + madd) - Dz  - (mhe * RHe * The * g) / (Ratm * Tatm) + m * g;
    fz = constrain(fz, 0.1, 0.25);

    int pw = map((int)(fz * 1000), 0, 250, 0, 255);  // Convert float to int for map()
    pw1 = pw;
    pw2 = 0;
    pw3 = 0;
    pw4 = pw;
    
  }
  else {
    Dz = 0;
    pw1 = 0;
    pw2 = 0;
    pw3 = 0;
    pw4 = 0;
  }
  analogWrite(IN1, pw1);
  analogWrite(IN2, pw2);
  analogWrite(IN3, pw3);
  analogWrite(IN4, pw4);
  
  while (micros() - LoopTimer < 10000);
  dt = (micros() - LoopTimer) / 1000000.0;
  LoopTimer = micros();
}

void normalizeBeliefRow(double* row, int len) {
  double sum = 0;
  for (int i = 0; i < len; i++) sum += row[i];
  if (sum > 0) {
    for (int i = 0; i < len; i++) row[i] /= sum;
  }
}

void updateBelief(double prex, double prey, double currentx, double currenty, int z_level) {
  const double alpha = 0.5;
  int z_idx = constrain(z_level - 1, 0, 4);

  double vx = currentx - prex;
  double vy = currenty - prey;

  int w_obs[2] = {0, 0};
  if (fabs(vx) >= 0.1) w_obs[0] = (vx > 0) ? 1 : -1;
  if (fabs(vy) >= 0.1) w_obs[1] = (vy > 0) ? 1 : -1;

  for (int i = 0; i < 4; i++) {
    if (possibleWinds[i][0] == w_obs[0] && possibleWinds[i][1] == w_obs[1]) {
      belief[z_idx][i] = (1.0 - alpha) * belief[z_idx][i] + alpha;
    } else {
      belief[z_idx][i] *= (1.0 - alpha);
    }
  }

  normalizeBeliefRow(belief[z_idx], 4);
}


int chooseAction(State s, double belief[5][4]) {
  if (random(1000) / 1000.0 < epsilon) {
    return random(3);  
  }

  int z_idx = constrain(s.z - 1, 0, 4);
  double best_value = -1;
  int best_idx = 0;
  for (int i = 0; i < 4; i++) {
    if (belief[z_idx][i] > best_value) {
      best_value = belief[z_idx][i];
      best_idx = i;
    }
  }
  return best_idx % 3;  
}

void getoptical() {
  while (opticalflowserial.available()) {
    uint8_t sensor_data = opticalflowserial.read();
    micolink_decode(sensor_data);
  }
}

void micolink_decode(uint8_t data) {
  static MICOLINK_MSG_t msg;

  if (!micolink_parse_char(&msg, data)) return;

  if (msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR) {
    MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
    memcpy(&payload, msg.payload, msg.len);
    dtOptical = 0.02;
    if (payload.distance < 8000 && fabs(payload.flow_vel_x) < 1500 && fabs(payload.flow_vel_y) < 1500) {
      if (dtOptical > 0.0) {
        zd = (payload.distance / 1000.0 - z) / dtOptical;
        z = payload.distance / 1000.0;

        xd = payload.flow_vel_x * z * 0.01;
        yd = payload.flow_vel_y * z * 0.01;

        x = x + xd * dtOptical;
        y = y + yd * dtOptical;
      }
    }
  }
}

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data) {
  switch (msg->status) {
    case 0:
      if (data == MICOLINK_MSG_HEAD) {
        msg->head = data;
        msg->status = 1;
      }
      break;
    case 1: msg->dev_id = data; msg->status = 2; break;
    case 2: msg->sys_id = data; msg->status = 3; break;
    case 3: msg->msg_id = data; msg->status = 4; break;
    case 4: msg->seq = data; msg->status = 5; break;
    case 5:
      msg->len = data;
      if (msg->len == 0) {
        msg->status = 7;
      } else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN) {
        msg->status = 0;
      } else {
        msg->payload_cnt = 0;
        msg->status = 6;
      }
      break;
    case 6:
      msg->payload[msg->payload_cnt++] = data;
      if (msg->payload_cnt == msg->len) {
        msg->status = 7;
      }
      break;
    case 7:
      msg->checksum = data;
      msg->status = 0;
      return true;
    default:
      msg->status = 0;
      msg->payload_cnt = 0;
      break;
  }
  return false;
}
