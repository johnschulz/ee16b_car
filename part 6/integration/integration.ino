/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_1
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_2

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_LISTEN;

#define DRIVE_FAR                   1
#define DRIVE_LEFT                  2
#define DRIVE_CLOSE                 0
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*    From closed_loop.ino   */
/*        with changes       */
/*---------------------------*/

float theta_left = 0.2356;
float theta_right = 0.2389;
float beta_left = -29.43;
float beta_right = -19.26;
float v_star      = 68.3;

// PWM inputs to jolt the car straight
int left_jolt  = 210;
int right_jolt = 240;

// Control gains
float k_left  = -0.7;
float k_right =  0.7;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*    From closed_loop.ino   */
/*---------------------------*/

float driveStraight_left(float delta)
{
    return ((v_star + beta_left) + (k_left*delta)) / theta_left;
}

float driveStraight_right(float delta)
{
    return ((v_star + beta_right) + (k_right*delta)) / theta_right;    
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*    From closed_loop.ino   */
/*---------------------------*/

float delta_ss = -0.5;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 91 // in cm - 6 feet diameter = 3 tiles in 125 Cory
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter = 2 tiles in 125 Cory

int run_times[4] = {2500, 6000, 3000, 3000};

float delta_reference(int k) {
  // YOUR CODE HERE
  if (drive_mode == DRIVE_RIGHT) {
    return CAR_WIDTH* v_star * k / 5.0 / TURN_RADIUS;
  }
  else if (drive_mode == DRIVE_LEFT) {
    return - CAR_WIDTH* v_star * k / 5.0 / TURN_RADIUS;;
  }
  else { // DRIVE_FAR, DRIVE_CLOSE
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int k) {
  // YOUR CODE HERE
  return 0;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                80
#define PRELENGTH                   5
#define THRESHOLD                   0.6

#define KMEANS_THRESHOLD            0.08
#define LOUDNESS_THRESHOLD          850

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

const float pca_vec1[SNIPPET_SIZE] = {-0.0439225573616, -0.0380358397714, -0.0253657452969, 0.000402681046817, 0.0121542163454, -0.0131871300096, -0.05879074584, -0.0327683417579, -0.0555949105533, -0.0755542014597, -0.0927659337464, -0.113085918586, -0.152698130974, -0.210264734965, -0.257231257409, -0.255110587014, -0.259563496339, -0.251039000087, -0.241049852818, -0.217551578753, -0.218346724729, -0.184294101302, -0.173040668176, -0.134974651229, -0.0898493624815, -0.0741170095477, -0.0561622545126, -0.0420966474614, -0.0238190519601, -0.00673705400277, -0.00788678417414, -0.0186399918382, -0.0168670870085, -0.0147960208248, -0.00644879318176, 0.00229295084341, 0.0206039769199, 0.0463004344656, 0.0691957939658, 0.0813229311196, 0.0935142143417, 0.0992800427478, 0.118202156709, 0.114347745005, 0.120132582575, 0.121952264888, 0.123229037952, 0.11922837454, 0.115119612138, 0.11208781296, 0.118613542592, 0.119796286378, 0.129569378527, 0.135236021684, 0.155311593037, 0.164660388414, 0.142551219019, 0.13060435984, 0.123509336522, 0.111376180152, 0.106364754503, 0.0929859391462, 0.0874436519069, 0.0794009786587, 0.0670438442162, 0.0660805953757, 0.0569335231172, 0.0465376449933, 0.0423871544606, 0.0314348395698, 0.0291376567117, 0.0221462866544, 0.0129860191637, 0.0067093051023, 0.00344734803435, 0.00395311689676, 0.00226645802865, 0.00188221880779, 0.00123478158351, 0.000684913512082};
const float pca_vec2[SNIPPET_SIZE] = {0.00289666382457, 0.00163976181001, -0.00728155881481, -0.00998768195023, -0.0431237107063, -0.116840458993, -0.141686881675, -0.119165235698, -0.0978577884826, -0.0704552983893, -0.0625799629642, -0.0406823012196, -0.0682847196668, -0.0992371686683, -0.0868995980945, -0.0698764539166, -0.0348983364335, -0.000878111988694, 0.0295613129443, 0.048002314666, 0.0500072317837, 0.0609079631189, 0.0819784814761, 0.122184693602, 0.156709004938, 0.180888804246, 0.172536199671, 0.153758655534, 0.107364803955, 0.0817600416699, 0.0681770185177, 0.0589024029313, 0.0530365167834, 0.0621806800604, 0.0585478122819, 0.0632632116719, 0.0625839778574, 0.0506605549408, 0.0340705916072, 0.041744915149, 0.0685381921917, 0.0874422020305, 0.140859624233, 0.143519639877, 0.154058347722, 0.181340042061, 0.19321920633, 0.19016511135, 0.178875007889, 0.159695746416, 0.155326622246, 0.116783401061, 0.0916862261738, 0.0319006874096, -0.0347228458782, -0.0705793123087, -0.117060508495, -0.14845097324, -0.196427005099, -0.224594907154, -0.239064155601, -0.222333869399, -0.201525866784, -0.181816852404, -0.163596149927, -0.156484728178, -0.139415804966, -0.114024096614, -0.10628012221, -0.0780949320597, -0.0717930588052, -0.0573273001942, -0.032915238464, -0.0163957685015, -0.0116251103706, -0.0112557352644, -0.0116031364419, -0.0100050754512, -0.00681938363627, -0.00282646692333};
//const float pca_vec3[SNIPPET_SIZE] = {-0.0715926189151, -0.0711995002383, -0.0915984934296, -0.10469933263, -0.135588921029, -0.280525832987, -0.290301995508, -0.269663581007, -0.24659872804, -0.250695289551, -0.259562810661, -0.241243677421, -0.167742178138, -0.0981551581744, -0.0318418721754, 0.000569260918303, 0.0483009404177, 0.0693254372714, 0.0856110317315, 0.0861472321981, 0.0953061723258, 0.126758318984, 0.10440815676, 0.0644809179483, 0.0708513469941, 0.0398029279059, 0.0303332273883, 0.0500305938027, 0.0437097682366, 0.0415236560001, 0.0679626072559, 0.0744465070728, 0.0767406408217, 0.0801504067342, 0.0656097036999, 0.0407124191724, 0.0133083038938, -0.0214510080989, -0.0673334909539, -0.0951014339047, -0.121453610323, -0.120047544694, -0.129879006351, -0.113593948098, -0.100802648961, -0.0898142125534, -0.0680184090984, -0.0420158600834, -0.0205583284167, 0.00588378448801, 0.00239551957242, 0.0108647223428, 0.00978895809508, 0.0213920360499, -0.00858621691107, -0.000630446882089, 0.00289274504913, 0.0322913151561, 0.0415949338942, 0.0525862631243, 0.0571891931007, 0.0645833675199, 0.0759778479523, 0.0935078074256, 0.0904166726904, 0.0939699390207, 0.0998834520222, 0.111929658367, 0.112225000909, 0.123750937685, 0.11596741105, 0.116119852606, 0.128864416969, 0.126351692409, 0.122067839699, 0.128299435809, 0.114142129495, 0.104969403052, 0.0961523569078, 0.0781478852403};
const float mean_vec[SNIPPET_SIZE] = {0.00473947132146, 0.00545304312499, 0.00776900899105, 0.0107512345322, 0.0165335056301, 0.0274798697974, 0.0285531046217, 0.028206067269, 0.0280612047996, 0.0279165448246, 0.0283065093154, 0.0276389383545, 0.0249869131547, 0.0210697677774, 0.0184717436495, 0.0164442440243, 0.0152873427758, 0.0143814212251, 0.0143089382618, 0.0141523292635, 0.0137665428114, 0.013272592613, 0.0136447171448, 0.0147989131346, 0.015533797358, 0.0159820485393, 0.0159658263782, 0.0148962725085, 0.0136630027222, 0.0132002916587, 0.0124622592953, 0.0113918776429, 0.0102395884605, 0.00914357163579, 0.00801766035691, 0.00744109856746, 0.00717348776941, 0.00729802213115, 0.0082715376964, 0.00877065951111, 0.00956765027255, 0.00963178011618, 0.0107102838938, 0.0103835779053, 0.0106686104318, 0.0110942942743, 0.011294036875, 0.0114080713832, 0.0111105430208, 0.0111147306876, 0.011623225296, 0.0120145218045, 0.0129347597323, 0.0133871746088, 0.0146976614555, 0.0152445036574, 0.014382634334, 0.0136995875169, 0.0133439652464, 0.0126859776541, 0.0124094594261, 0.0115124435243, 0.0111437081424, 0.0105291646414, 0.00974954902486, 0.00939487415557, 0.00896346634132, 0.00806468727442, 0.00770009788176, 0.00684555216737, 0.00651005590412, 0.00591696058848, 0.00537294564124, 0.00484739490094, 0.00449096656602, 0.00456500649107, 0.00432174647584, 0.00393128972519, 0.00381793224527, 0.00347013806639};
float centroid1[3] = {-0.10255260969, -0.004121310866};
float centroid2[3] = {-0.0116559947513, -0.0141774311723};
float centroid3[3] = {0.0222897903873, 0.0423898183223};
float centroid4[3] = {0.0130945099088, -0.0638838304231};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int i = 0; i <= 4; i++) {
    sample_lens[i] = run_times[i]/SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  start_listen_mode();
}

void loop(void) {
  check_encoders();
  if (timer_mode == MODE_LISTEN && re_pointer == SIZE){
    // Stop motor
    write_pwm(0, 0);
    digitalWrite(RED_LED, LOW);

    // if enveloped data is above some preset value
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/

      // Perform principal component projection
      for(int i = 0; i < SNIPPET_SIZE; i++){
        result[i] -= mean_vec[i];
      }

      // Dot products
      for(int i = 0; i < SNIPPET_SIZE; i++){
        proj1 += result[i] * pca_vec1[i];
      }
      for(int i = 0; i < SNIPPET_SIZE; i++){
        proj2 += result[i] * pca_vec2[i];
      }


      // Classification
      // Use the function l2_norm defined above
      // ith centroid: centroids[i]
      float mini[4] = {0,0,0,0};
      float m = 100000.0;
      unsigned word = 10;
      for(int i = 0; i < 4; i++){
        mini[i] = l2_norm(proj1, proj2, centroids[i]);
        if (m > mini[i]){
          m = mini[i];
          word = i;
        }
        Serial.print("distance to ");
        Serial.print(i);
        Serial.print(" is ");
        Serial.println(mini[i]);
        
      }

      // Check against KMEANS_THRESHOLD and print result over serial
      // YOUR CODE HERE
      if (m < KMEANS_THRESHOLD) {
        switch(word){
          case 0: Serial.println("Slow"); break;
          case 1: if (mini[3] - mini[1] < 0.03) 
                  {
                    word = 3;
                    Serial.println("Read Forward, but choosing right.");
                  }
                  else
                  {
                    Serial.println("Forward");
                  }
                  break;
          case 2: Serial.println("Left"); break;
          case 3: Serial.println("Right"); break;
        }
        drive_mode = word; // from 0-3, inclusive
        start_drive_mode();
        
      }
      else{
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    delay(2000);
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    re_pointer = 0; // start recording from beginning if we don't start driving
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta - delta_reference(step_num) - straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

void start_listen_mode(void) {
  re_pointer = 0;
  write_pwm(0, 0);
  delay(3000); // 3 seconds buffer for mic cap settling
  timer_mode = MODE_LISTEN;
  setTimer(MODE_LISTEN);
}

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.2*4096))
#define HIGH_THRESH                 ((int) (0.8*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TA2CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TA2CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TA2CCTL0 = CCIE; // enable interrupts for Timer A
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
