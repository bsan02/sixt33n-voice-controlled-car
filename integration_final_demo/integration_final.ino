/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

#define LEFT_MOTOR                  6
#define LEFT_ENCODER                3
#define RIGHT_MOTOR                 5
#define RIGHT_ENCODER               2

#define RXLED                       17
#define TXLED                       30

#define SAMPLING_INTERVAL           100
#define NUM_COMMANDS                4
int sample_lens[NUM_COMMANDS] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1

#define NUM_COMMANDS                4
#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;

volatile int left_count = 0;
volatile int right_count = 0;

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*    From closed_loop.ino   */
/*---------------------------*/
float theta_left = .3811;
float theta_right = .4371;
float beta_left = -16.8;
float beta_right = -9.645;
float v_star = 64.4;

// PWM inputs to jolt the car straight
int left_jolt = 220;
int right_jolt = 240;

// Control gains
float f_left = 0.8;
float f_right = 0.8;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/
float driveStraight_left(float v_star, float delta) {
  return 1 / theta_left * (v_star - f_left * delta + beta_left);
}

float driveStraight_right(float v_star, float delta) {
  return 1 / theta_right * (v_star + f_right * delta + beta_right);
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/

float delta_ss = 0;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 91 // in cm - 6 feet diameter
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter

/*---------------------------*/
/*    PREPROGRAMMED PATH     */
/*---------------------------*/
int run_times[NUM_COMMANDS] = {7000, 850, 2500, 850}; // length of commands roughly in ms
int drive_modes[NUM_COMMANDS] = {DRIVE_FAR, DRIVE_LEFT, DRIVE_CLOSE, DRIVE_RIGHT};

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
float delta_reference(int i) {
  // YOUR CODE HERE
  if (drive_mode == DRIVE_RIGHT) { // Return a NEGATIVE expression
    return -(CAR_WIDTH * v_star * i) / TURN_RADIUS;
  }
  else if (drive_mode == DRIVE_LEFT) { // Return a POSITIVE expression
    return (CAR_WIDTH * v_star * i) / TURN_RADIUS;
  }
  else { // DRIVE_FAR, DRIVE_CLOSE
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON6      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int i) {
  // YOUR CODE HERE
  return 0; // Replace this line
}

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

#define SIZE                        5504
#define ADC_TIMER_MS                0.35
#define AVG_SHIFT                   5
#define AVG_SIZE                    (int) pow(2, AVG_SHIFT)
#define SIZE_AFTER_FILTER           (int) SIZE / AVG_SIZE

#define MIC_INPUT                   A2

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/

#define SNIPPET_SIZE                80
#define PRELENGTH                   5
#define THRESHOLD                   0.6
#define BASIS_DIM                   3

#define EUCLIDEAN_THRESHOLD         0.04
#define LOUDNESS_THRESHOLD          140

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/

float pca_vec1[SNIPPET_SIZE] = {-0.02422089316375582, -0.03973011458693243, -0.05776484640722079, -0.05869355458328007, -0.09527032711486379, -0.07259418383938339, -0.14079148028913166, -0.2070339845544902, -0.26981271178528654, -0.292265323787181, -0.3036660404228801, -0.30786394074599255, -0.28326651700446664, -0.2515076287143501, -0.1890233084611615, -0.1513819450398698, -0.08803947985535863, -0.05729970957642113, -0.041819715064700946, 0.028249249248444134, 0.08353957063527677, 0.10246313504423395, 0.1500837667266214, 0.1653709521494207, 0.21261046951955334, 0.2311714787905691, 0.2222264031676069, 0.18860832494999366, 0.14984086010766454, 0.11285778696845171, 0.09261723797825637, 0.06290452166858031, 0.053747786244544536, 0.04078260037138861, 0.047328828533428347, 0.03974695598152996, 0.03783673311265898, 0.05188107359853568, 0.05432004295740696, 0.05685637530330304, 0.05952922460854642, 0.06112174356892712, 0.06007533472172324, 0.058240624120446845, 0.060494065980877254, 0.05796567694314117, 0.05448787757585557, 0.049821968184590004, 0.05069733574970192, 0.03420560144214232, 0.04047816318016626, 0.03923262161449029, 0.02688865891565807, 0.015006381649691676, 0.02104566544277354, 0.013232394979302828, 0.011839380588626549, 0.007699842953315575, 0.013045460480284361, 0.00848939401122621, 0.0014343144172180302, 0.002397911253804533, 0.002373686242424798, 0.0010177216172037378, 0.003316085173720705, 0.0009572449315733302, 0.0011525441610065886, 0.00010181144144671874, 0.0009432437235999575, 0.0012033861864818348, 0.00015871890057987334, 3.37964301138683e-05, 0.00016808712165089352, -0.0007423652330781659, -0.0032718189501052823, -0.0011866605888974917, -0.0024534175164204765, -0.0021678218413209937, -0.00133584676566682, -0.0006964854775644669};
float pca_vec2[SNIPPET_SIZE] = {0.04916988451893113, 0.052281783186051256, 0.05557198803276412, 0.09663474322249421, 0.09474603629781225, 0.14881861175457461, 0.19059430998406435, 0.19958940219735852, 0.17520856290482606, 0.11454652779399291, 0.03922383418115365, -0.016930155705155145, -0.07684278738653502, -0.1443461217199247, -0.19416428120756055, -0.289206577136388, -0.29503464675576685, -0.3248309062295247, -0.3200079875651716, -0.3026356157466312, -0.279158368924971, -0.203728845938979, -0.10883777253940594, -0.06769979171751746, -0.06421205724894166, -0.06873615458710387, -0.010655905201546481, 0.03953858034702145, 0.07645400847445494, 0.12180464583099572, 0.15022545608235502, 0.15458719787666078, 0.13873522982529587, 0.1254639312774637, 0.1069034333120478, 0.09151992408051517, 0.08426985117118284, 0.06458773845490164, 0.06520034218714169, 0.061720912412895, 0.05399796388840342, 0.04288452527235853, 0.03472221985462806, 0.021910365112922586, 0.014322492627186118, 0.007371421125243386, 0.002309976995876928, 0.00581336938306357, 0.011543252521141297, 0.003639804696677431, -0.0011522152213473357, 0.00029351637475907025, 0.003983470733686041, 0.005805633927610526, 0.0035989795388798635, 0.004221965703813332, 0.004482506381939416, 0.0033015854720622817, 0.0035609555131021747, 0.0037211822122237997, 0.0040160861788591466, 0.001224454634754504, -0.00012499839580619435, 0.00035783479526127856, 0.0013898531519570161, 0.002325650400382714, 0.0026027005149143983, 0.004238817279459356, 0.002856781887087666, 0.0034858011614201886, 0.003654900401693642, 0.005794938174008556, 0.005873516648192072, 0.002137410226889774, 0.0017200485086252702, 0.0010048607018370858, -0.0015620654906518025, 0.0005844427169071692, -0.0028382523279321123, 0.0005552869220779972};
float pca_vec3[SNIPPET_SIZE] = {-0.03945768770266632, -0.04566966573115305, -0.07566329308202163, -0.11197781384773726, -0.13467282724589558, -0.2925202859781304, -0.31211356386910355, -0.25127545200271534, -0.21459428578208184, -0.08330275664515438, 0.045384230013307686, 0.07432518530749246, 0.14555374370582674, 0.1361223764892407, 0.16036690479586932, 0.14011214234271568, 0.058554071893090365, -0.01767697120025345, -0.07053380339975299, -0.11406397528719969, -0.12299083514919174, -0.2396103752497093, -0.25006224217107914, -0.20285912723010827, -0.11298860197010174, -0.06396728357366062, -0.04685825315734266, -0.05749721700657185, -0.04234660471367259, 0.019426383456561957, -0.010080461917761827, 0.04197383812113424, 0.1336190412741311, 0.14572821429148813, 0.1845558371077149, 0.19902303728930457, 0.20695095416883144, 0.20790344642320852, 0.1703851632998587, 0.16544329988841933, 0.17277824251767596, 0.1389522380723726, 0.11276437808910202, 0.12429827953537231, 0.06816827818937132, 0.05495700367890339, 0.023998443410937927, 0.008428311363849632, 0.026547695116986243, 0.011866801001405776, -0.009554038436836845, -0.017178242353325317, 0.037266388797036906, 0.002920827710976647, -0.0022093516986909844, 0.0003041525660651559, 0.0007952134700816209, 0.0010451659065482104, -0.006236525635345169, 0.0001698860553159248, 0.001310789227166206, -0.004437519445131021, -0.00668451977282664, -0.006029835115650246, -0.004938708857041482, -0.004453872493731579, -0.002123252348695527, -0.002517893156543505, -0.003682498309523768, -0.002857877680478188, -0.0030968238616311777, -0.0025367964526444263, -0.004313242895406585, -0.003306606315007621, -0.00304938929686252, -0.0022837036107481853, -0.005352760897498311, -0.0051834148579740745, -0.0033266623109554252, -0.003863044861750044};
float projected_mean_vec[BASIS_DIM] = {-0.07779114588258842, 0.007565281374330856, -0.04488981095460775};
float centroid1[BASIS_DIM] = {-0.02483104632681398, 0.01235705975533543, 0.03436949945193712};
float centroid2[BASIS_DIM] = {-0.03695555313127201, -0.05362996277702999, -0.013180552267443064};
float centroid3[BASIS_DIM] = {0.07161687234558989, -0.002992843787407956, -0.006078379490415933};
float centroid4[BASIS_DIM] = {-0.013404917573868995, 0.060362382012412515, -0.020605319582833806};
float* centroids[4] = { // DO NOT DELETE THIS CHUNK
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

//data array and index pointer
int16_t out[SIZE_AFTER_FILTER] = {0};
volatile int re_pointer = 0;

int16_t re0[AVG_SIZE] = {0};
int16_t re1[AVG_SIZE] = {0};
int write_arr = 0;

int16_t * get_re(int loc){
  switch(loc){
    case 0:
      return re0;
    case 1:
      return re1;
    default:
      return re0;
  }
}

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;
float proj3 = 0;

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
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);
  delay(500);

  re_pointer = 0;
      
  cli();
  //set timer1 interrupt at 1Hz * SAMPLING INTERVAL / 1000
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15.624 * ADC_TIMER_MS;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();

  write_pwm(0, 0);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(TXLED, LOW);
  digitalWrite(RXLED, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(TXLED, HIGH);
  delay(1000);
  digitalWrite(RXLED, HIGH);
  delay(1000);

  for (int i = 0; i < 4; i++) {
    sample_lens[i] = run_times[i] / SAMPLING_INTERVAL;
  }

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), flag_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), flag_left, CHANGE);
  
  start_listen_mode();
}

void loop(void) {
  if (re_pointer%AVG_SIZE == 0){
    write_arr = !write_arr;
    envelope_small(get_re(!write_arr), out, re_pointer>>AVG_SHIFT);
  }
  if (re_pointer == (int) (SIZE / 3)) {
    digitalWrite(TXLED, LOW);
  }
  if (re_pointer == (int) (SIZE * 2 / 3)) {
    digitalWrite(RXLED, LOW);
  }
  if (loop_mode == MODE_LISTEN && re_pointer == SIZE) {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(TXLED, HIGH);
    digitalWrite(RXLED, HIGH);
    write_pwm(0, 0);
    
    // if enveloped data is above some preset value
    if(envelope(out, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;
      proj3 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
        proj1 += result[i] * pca_vec1[i];
        proj2 += result[i] * pca_vec2[i];
        proj3 += result[i] * pca_vec3[i];
      }

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];
      proj3 -= projected_mean_vec[2];

      float best_dist = 999999;
      int best_index = -1;
      
      // Classification
      // Use the function l2_norm3 defined above
      // jth centroid: centroids[j]
      for (int i = 0; i < 4; i++) {
//        Serial.print(i);
//        Serial.print(" ");
//        Serial.println(l2_norm3(proj1, proj2, proj3, centroids[i]));
        if (l2_norm3(proj1, proj2, proj3, centroids[i]) < best_dist) {
          best_dist = l2_norm3(proj1, proj2, proj3, centroids[i]);
          best_index = i;
        }
      }
      

      // Check against EUCLIDEAN_THRESHOLD and execute identified command
      // YOUR CODE HERE
      if (best_dist < EUCLIDEAN_THRESHOLD) {
        drive_mode = drive_modes[best_index];
        start_drive_mode();
      }
   } else {
     Serial.println("Below LOUDNESS_THRESHOLD.");
   }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
    
  } else if (loop_mode == MODE_DRIVE) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    } else {
      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_count;
      int right_position = right_count;

       /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta + delta_reference(step_num) + straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(v_star, delta);
      int right_cur_pwm = driveStraight_right(v_star, delta);
      write_pwm(left_cur_pwm, right_cur_pwm);
      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
      
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;
    digitalWrite(RXLED, (!(((drive_mode == DRIVE_FAR) || (drive_mode == DRIVE_CLOSE) || (drive_mode == DRIVE_RIGHT)) && ((step_num / 5) % 2))));
    digitalWrite(TXLED, (!(((drive_mode == DRIVE_FAR) || (drive_mode == DRIVE_CLOSE) || (drive_mode == DRIVE_LEFT)) && ((step_num / 5) % 2))));

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }
    delay(SAMPLING_INTERVAL);
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void envelope_small(int16_t* data, int16_t* data_out, int index){
  int32_t avg = 0;
  for (int i = 0; i < AVG_SIZE; i++) {
      avg += data[i];
  }
  
  avg = avg >> AVG_SHIFT;
  data_out[index] = abs(data[0] - avg);  
  
  for (int i = 1; i < AVG_SIZE; i++) {
      data_out[index] += abs(data[i] - avg);
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    Serial.println(maximum);
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres && block < SIZE_AFTER_FILTER);
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

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void flag_left() {
  if(digitalRead(LEFT_ENCODER)) {
    left_count++;
  }
}

void flag_right() {
  if(digitalRead(RIGHT_ENCODER)) {
    right_count++;
  }
}

void start_drive_mode(void) {
  loop_mode = MODE_DRIVE;
  step_num = 0;
  left_count = 0;
  right_count = 0;
}

void start_listen_mode(void) {
  write_pwm(0, 0);
  delay(3000);
  loop_mode = MODE_LISTEN;
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

ISR(TIMER1_COMPA_vect){//timer1 interrupt 8Khz toggles pin 13 (LED)
  if (re_pointer < SIZE && loop_mode != MODE_DRIVE) {
    digitalWrite(RXLED, LOW);
    get_re(write_arr)[re_pointer%AVG_SIZE] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}
