#include<stdio.h>
#define _USE_MATH_DEFINES//VisualStudio�p
#include<math.h> 
#include<time.h>
#include<stdlib.h>

enum methods {WALK, BIKE, CAR, CAR_H, TRAIN};
enum goals {STATION_A, STATION_B, COMPANY, HIGHWAY_S, HIGHWAY_E, PARKING, BIKELOT, STATION_LOT};
enum routs {W_STATION_A, W_COMPANY, B_STATION_A, B_COMPANY, B_BIKELOT, C_COMPANY, C_HIGHWAY_S, C_PARKING, C_HIGHWAY_E, T_STATION_B, B_STATION_LOT,END = 100};

enum methods method;
enum goals goal;
enum routs routs;
 
#define NUM_AGENT 1000                         //�G�[�W�F���g�� 
#define NUM_HOME  1000                       //�Ƃ̐�
#define NUM_COMPANY 1000                        //��Ђ̐�
#define NUM_PARKING 1000                          //���ԏ�̐�(�����l)
#define NUM_BIKELOT 1000                          //���֏�̐�
#define NUM_ROUTE_TYPE 8                          //�g���b�v�̎�ސ�
#define MAX_ROUTE_DEPTH 10                          //��i�g���b�v�̍ő吔(���ۂɎg���Ă�̂�4�܂�)

#define Xl -6000                       //���z��Ԃ�X���Œ�l-6000
#define Xh 6000                        //�@�@�@�@�@�@ �ő�l6000
#define Yl -4000                       //���z��Ԃ�Y���Œ�l-4000
#define Yh 4000                        //�@�@�@�@�@ �@�ő�l4000

/////////////////////////////////////////////////////////////////////////////
#define MM 0                           //���r���e�B�}�l�W�����g�̊���(0-1)
double charge_g = 0.1; //0.01 ;                  //�K�\������(�~/m)
/////////////////////////////////////////////////////////////////////////////

double L = 4000;                         //�Q�w�Ԃ̋���(m)
double Vw = 66;                          //�������x(m/��)(4km/h)
double Vb = 267;//160;                         //���]�Ԃ̑��x(m/��)(9.6km/h)
double Vc = 450;                         //�����Ԃ̑��x(m/��)(27km/h)
double Vch = 833;                        //�����Ԃ̊������H�̑��x(m/��)(50km/h)
double Vt;                               //�d�Ԃ̑��x
double Lb= 3200;                         //businesszone�̔��a 
double Lh= 3200;                         //homezone�̔��a
int route[NUM_ROUTE_TYPE][MAX_ROUTE_DEPTH];    //���[�g

int car_dencity[(Yh-Yl)/100][(Xh-Xl)/100];              //��悲�Ƃ̎Ԗ��x
int home_dencity[(Yh-Yl)/100][(Xh-Xl)/100];             //��悲�Ƃ̎���x
int com_dencity[(Yh-Yl)/100][(Xh-Xl)/100];              //��悲�Ƃ̉�Ж��x

double E = 0.7;                          //�ψيm���̏����l
double G = 0.94;                         //�ψيm���Ɋ|����萔
double B = 0.5;                          //�w�K�W��

double P_remove = 0.5;                   //���]�ԓP���m��
double charge_rm = 3000;                 //�P���Ԋґ�
double charge_t;                          //�d�ԑ�
  
//double count_CAR_H = 0;
//double count_CAR = 0;
int    count_par =0;                     //�|�Y�������ԏ�̐�

//�ϊ��ϐ�
double change_f = 20;                   //���
double change_t = 30;                   //����

int route[NUM_ROUTE_TYPE][MAX_ROUTE_DEPTH];    //���[�g 
int check;                               //�G�[�W�F���g�����m�F�̂��߂̕ϐ�


  //�ړ���i�ƖړI�n
struct unit_trip{
  int method;                           //���@
  int goal_id;                          //���݂̖ړI�n

};
                                      
struct trip{
  int unit[10];                         //�g���b�v��[]�Ԗڂ̎�i�g���b�v       
  int unit_state;                       //�G�[�W�F���g�͑I�������g���b�v��unit_state�Ԗڂ̎�i�g���b�v���g���Ă���
  int route_num;                        //�G�[�W�F���g���I�������g���b�v�ԍ�
};


//�G�[�W�F���g
struct agent{
  double x[2];                     //�G�[�W�F���g�̈ʒu 
  double W[3];                     //�G�[�W�F���g�̕Ό�(����ǂ�,��,����)
  double route_cost[NUM_ROUTE_TYPE]; //�e�g���b�v�ɂ�����R�X�g
  struct trip t;                   //�g���b�v      
  double v;                        //���݂̑��x
  double time;                     //�ړ�����
  double distance_c;               //�ړ�����(��)
  double distance_w;               //�ړ�����(�k��)
  double distance_b;               //�ړ�����(���]��)
  double distance_t;               //�ړ�����(�d��)
  double money;                    //��p
  double fatigue;                  //���
  int company;                     //�ړI�n�i���o�[
  int parking;                     //���p���Ă��钓�ԏ�̃i���o�[
  int bikelot;                     //�@�@�@�@�@�@���֏�̃i���o�[

	int d_num[2];                    //���ݒn�]�[���i���o�[(�G�[�W�F���g�̍�������)
};


//�Z��
struct home{
  double x[2];                     //�Ƃ̍��W
  double price;                    //�n��
};

//���]����
struct home_temp{
	double x[2];                      //���]����̍��W
	double price;                     //�n��
	//double cost;
};

//���
struct company{
  double x[2];                     //��Ђ̍��W 
};

//��悲�Ƃ̖��x
struct density{                   
  double car;                      //�Ԗ��x
	double home;                     //����x
	double com;                      //��Ж��x
	double Hprice;                   //��悲�Ƃ̒n��
};

//���ԏ�
struct parking{
  double x[2];                    //���ԏ�̍��W 
  int capacity;                    //�e��
  int charge;                      //���ԑ�
  int num_park;                    //���ԑ䐔
};

//���֏�
struct bikelot{   
  double x[2];                    //���֏�̍��W
  int capacity;                    //�e��
  int charge;                      //���֑�
  int num_lot;                     //���֑䐔
};                                   
//�w�̒��֏�
struct stationlot{
 double x[2];                    //�w���֏�̍��W
  int capacity;                  //�e��
  int charge;                    //���֑�
  int num_lot;                   //���֑䐔
};  

//�w
double station_a[2];             //�wA�̍��W
double station_b[2];             //�wB�̍��W

//struct density den[500][500];

//�֐� 
//�G�[�W�F���g,�Z��,��ЂȂǂ̏����ݒ�
//����������W������
void setup_home(struct home h[]);
//���]����(����)������
void reset_home(struct home h[],struct home_temp ht[]);
//�n��v�Z
void home_price(struct home h[],struct home_temp ht[],struct company c[]);
//�]���挈��
void select_home(struct home h[],struct home_temp ht[],int counter,FILE *fp8);

//���r���e�B�}�l�W�����g
void mobility_management(struct agent a[]);

//��Ѝ��W������
void setup_company(struct company c[]);
//�e�g���b�v���̎�i�g���b�v������
void setup_trip(struct unit_trip u[]);
//�G�[�W�F���g�̏�����
void setup_agent(struct agent a[], struct home h[]);
//���ԏ�E���֏���W������
void setup_park_lot(struct parking p[], struct bikelot l[], struct stationlot sl);

//�G�[�W�F���g���ׂĈړ��I���܂Ń��[�v
void MoveToDestination(struct agent a[],struct unit_trip u[],struct company c[],struct parking p[],struct bikelot bl[],double next[]);

//�G�[�W�F���g�̃��[�g�I��
void select_rute(struct agent a[]);                             
//double select_trip();   //�ړ���i�̌���                                                                                                                     
//�G�[�W�F���g�̍ď�����
void reset_agent(struct agent a[], struct home h[]);
// �G�[�W�F���g�̌��݂̑��x�̌v�Z                                      
double speed(int trip);                                   
//�G�[�W�F���g�̈ړ�                           
void movement(struct agent a[],double next[], struct unit_trip u[], int i);                                                            
//���̖ړI�n�̌���
void next_spot(struct agent a[], struct company c[], struct parking par[], struct bikelot bl[], struct unit_trip u[], double next[], int i);       
//�����m�F
int check_arrival(struct agent a[], double next[], int i);                                                         
//�ړ��ɂ����������Z�o
void tired(struct agent a[], struct unit_trip u[], int i);
//�ړ��ɂ������������Z�o    
void money(struct agent a[], struct parking p[], struct bikelot l[], struct stationlot sl, int i);                
//�ړ����R�X�g�Z�o
void cost(struct agent a[], int i);
//���]����ł̈ړ����R�X�g�Z�o
double cost_home(struct agent a[], int i);
//�Ԗ��x�̌v�Z
void calc_dencity(struct agent a[], struct unit_trip u[]);
//�a�؂ɂ�鑬�x�����E���̉��Z
void congestion(struct agent a[], struct unit_trip u[], int i);
//���֏�I��
int selectbiklot(struct company c[], struct agent a[], struct bikelot bl[], int i);
//���ԏ�I��
int selectparking(struct company c[], struct agent a[], struct parking p[], int i);


main()
{
	//�錾������  
  struct unit_trip unit[20];                        //�g���b�v  
  struct agent age[NUM_AGENT];                              //�G�[�W�F���g
  struct home hom[NUM_HOME];                               //�z�[��
	struct home_temp homt[1000];                              //���]����
  struct company com[NUM_COMPANY];                            //�ړI�n
  struct parking park[NUM_PARKING];                           //���֏�
  struct bikelot lot[NUM_BIKELOT];                            //���ԏ�
  struct stationlot s_lot;                                    //�w���֏�

  int i , h,j,k,l,n,q;                                     //���[�v
  int counter;
  int x;                                                   //���s��
  
  double next[2];                                          //�G�[�W�F���g�̌��݂̖ڕW���W
  int count[NUM_ROUTE_TYPE] = {0};                           //�e�g���b�v��I�������G�[�W�F���g��
	
	//int counter2[101][101]={0};
	//int counter_m = 0;

	//int car_dencity[][1000]={0};//�Ԗ��x[Lh*2/100][L+Lh+Lb]


  //int distance_a[NUM_ROUTE_TYPE] = {0};//���ϋ���(����-�wA)
  //int distance_b[NUM_ROUTE_TYPE] = {0};//���ϋ���(����-�ړI�n)

  FILE *fp1;  
  FILE *fp2;
  FILE *fp3;
  FILE *fp4;

	FILE *fp7;
	FILE *fp8;
	FILE *fp9;
	
	FILE *fp10;
	FILE *fp11;
	FILE *fp12;

  charge_t = L * 0.04;//�d�ԑ�
  station_a[0] = -L/2;   station_a[1] = 0;//�w���W
  station_b[0] =  L/2;   station_b[1] = 0;
   //printf("%f\n",station_b);

  s_lot.x[0]=s_lot.x[1]=0;
  s_lot.capacity=0;
  s_lot.charge=0;
  s_lot.num_lot=0;

	
/*
  if((fp1 = fopen("data1000.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }
*/
  if((fp2 = fopen("L500.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }
/*
  if((fp3 = fopen("route.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }
*/
  if((fp4 = fopen("trip_change.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }

	//���z�}
  if((fp7 = fopen("home1.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }
  
  if((fp8 = fopen("Move_Cost.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }
  
  if((fp9 = fopen("price_routecost.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }
  
  if((fp10 = fopen("all_cost.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }
  
  if((fp11 = fopen("co2.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }
  
  if((fp12 = fopen("need_place.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }
  
  srand((unsigned)time(NULL));       /* �����̏����� */

  //������
  setup_trip(unit);
  setup_home(hom);
  setup_company(com);
  setup_agent(age, hom);
  setup_park_lot(park,lot,s_lot);

//�d�ԑ��x�Z�o
// L=5000 �Ƃ���� Vt =1100m/min = 66km/h ���炢���D
  Vt = L/7 +350;
  if(Vt>1100)
    Vt=1100;

//��ʍs���I��

  printf("��ʍs�����s��:");
  scanf("%d",&x);
  
  //���s�񐔃��[�v
  for(h=0;h<x;h++){
  
    //�ēx�������K�v�ȕ�������
    reset_agent(age, hom);
    
    //���ԏ�̉��z�e�ʐݒ�
    for(i=0;i<NUM_PARKING;i++){
      park[i].capacity = 4;
    }
    
    printf("-------------%d����--------------\n",h);
    
     //�G�[�W�F���g���ׂĈړ��I���܂Ń��[�v
    MoveToDestination(age,unit,com,park,lot,next);

    //�ړ����R�X�g�̎Z�o
    for(k=0;k<NUM_AGENT;k++){
      money(age, park, lot, s_lot, k);    
      cost(age, k);
    }
    
    //���Ԑ����O�Ȓ��ԏ�|�Y.�|�Y����capacity=0
    for(i=0;i<NUM_PARKING;i++){
      if(park[i].num_park == 0)
        park[i].capacity = 0;
    }
    
    //�G�[�W�F���g�̃g���b�v�I��
    if(h != (x-1))
      select_rute(age);
  
     //10�G�[�W�F���g��1�����Ƃ̃g���b�v�I��ω�
    for(i=0;i<10;i++){
      fprintf(fp4,"%d %d  ",i,age[i].t.route_num+1);
     }
    fprintf(fp4,"\n");
    
  }
  
  for(i=0;i<NUM_AGENT;i++){
   // printf("%d  x=%f y=%f time %f tired %f route = %d cost = %f\n",i, age[i].x[0], age[i].x[1], age[i].time, age[i].fatigue, age[i].t.route_num, age[i].route_cost[age[i].t.route_num]);
    count[age[i].t.route_num]++;
  }

  //�e�g���b�v���Ƃ̃G�[�W�F���g�����o��
  for(i=0;i<NUM_ROUTE_TYPE;i++){
    fprintf(fp2,"%d %d\n",i+1, count[i]);
  }
/*
  for(i=0;i<NUM_PARKING;i++){//���ԏꂲ�Ƃ̒��ԑ䐔
    if(park[i].capacity != 0){
      fprintf(fp1,"%4d %3d %.0lf\n",i,park[i].num_park,(double)(park[i].num_park/park[i].capacity)*100);
      count_par++;
    }
  }
  fprintf(fp1,"\n%d/%d  %.0lf",(NUM_PARKING-count_par),NUM_PARKING,(double)(NUM_PARKING-count_par)/NUM_PARKING*100);//�c�������ԏ�̐�
*/





//���Z�n�I��

	//���r���e�B�}�l�W�����g
  mobility_management(age);

  for(i=0;i<NUM_ROUTE_TYPE;i++){
    count[i]=0;
  }
  for(i=0;i<NUM_AGENT;i++){
    count[age[i].t.route_num]++;
  }
 
  fprintf(fp2,"\n");
      
  //�e�g���b�v���Ƃ̃G�[�W�F���g��
	for(i=0;i<NUM_ROUTE_TYPE;i++){
    fprintf(fp2,"%d %d\n",i+1, count[i]);
  }
   
  //�]����
  printf("�]�����s��:");
  scanf("%d",&x);

  fprintf(fp9,"i homt_price,cost_home time money fatigue\n");
  
  //100�G�[�W�F���g���]��,����͂��ꂼ��10�ӏ�
	for(h=0;h<x;h++){
		for(counter=0;counter<NUM_AGENT;counter+=100){
			printf("\n%dcycle%d�`%d�G�[�W�F���g�]��\n",h,counter,counter+100);

      //�]�����I��
			reset_home(hom,homt);
			//�n��v�Z
			home_price(hom,homt,com);

			//��ʈړ�
			for(k=0;k<1000;k+=100){

        //�ēx�������K�v�ȕ�������
				reset_agent(age, hom);

        //100�G�[�W�F���g�����]��
				for(j=counter;j<counter+100;j++){
					age[j].x[0]=homt[(j-counter)+k].x[0];
					age[j].x[1]=homt[(j-counter)+k].x[1];
					//printf("%d %lf %lf\n",j,age[j].x[0],age[j].x[1]);
				}

        //���]���悩��ړI�n�ւ̈ړ�
       MoveToDestination(age,unit,com,park,lot,next);

				//�R�X�g�̎Z�o
				for(j=counter;j<counter+100;j++){
					money(age, park, lot, s_lot, j);
					fprintf(fp9,"%d %.0lf %.0lf %.0lf %.0lf %.0lf\n",(j-counter)+k,homt[j].price,cost_home(age,j),age[j].time*	change_t,age[j].money,age[j].fatigue*change_f);
					
					//�]���R�X�g=�n��+�ړ��R�X�g    
					homt[(j-counter)+k].price = homt[(j-counter)+k].price/360 + cost_home(age,j);
				}
	
			}
      
      //�]����I��+�]��
			select_home(hom,homt,counter,fp8);
		   
			fprintf(fp7,"\n%dcycle%d�`%d�G�[�W�F���g�]��\n",h,counter,counter+100);
			fprintf(fp9,"\n%dcycle%d�`%d�G�[�W�F���g�]��\n",h,counter,counter+100);
			fprintf(fp8,"\n%dcycle%d�`%d�G�[�W�F���g�]��\n",h,counter,counter+100);
			for(i=0;i<NUM_HOME;i++){
				fprintf(fp7,"%d %.0lf %.0lf\n",i,hom[i].x[0],hom[i].x[1]);
			}
		
		}
	}
	
//�S�G�[�W�F���g�]���I����̑��R�X�g�{CO2�Z�o

	//�n��v�Z	
	home_price(hom,homt,com);
	
	//�ēx�������K�v�ȕ�������	reset_agent(age, hom);
	
	//�]���悩��ړI�n�ւ̈ړ�
  MoveToDestination(age,unit,com,park,lot,next);

	//�R�X�g�̎Z�o
	for(i=0;i<NUM_AGENT;i++){
	  money(age, park, lot, s_lot, i);
	  fprintf(fp10,"%d, %.0lf, %.0lf, %.0lf, %.0lf, %.0lf, %.0lf\n",i,hom[i].price,cost_home(age,i),age[i].time*change_t,age[i].money,age[i].fatigue*change_f,hom[i].price/360+cost_home(age,i));
	  fprintf(fp11,"%d, %.2lf, %.2lf, %.2lf, %.2lf\n",i,age[i].distance_c/1000*100,age[i].distance_w/1000*0,age[i].distance_b/1000*0,age[i].distance_t/1000*30);//co2=�ړ������~A walk��bike��0
	  fprintf(fp12,"%d, %.2lf, %.2lf, %.2lf, %.2lf\n",i,age[i].distance_c/1000*100,age[i].distance_w/1000*2,age[i].distance_b/1000*8,age[i].distance_t/1000*6);//�K�v���=�ړ������~A
	  //homt[(j-counter)+k].price = homt[(j-counter)+k].price/360 + cost_home(age,j);// *50/365 �ړ������/1000
	}

/*
//��ʍs���I��2���
  
  fprintf(fp2,"\n--2--\n\n");
  
  //������
  E=0.7;  
  for(i=0;i<NUM_AGENT;i++){
    for(k=0;k<NUM_ROUTE_TYPE;k++){
      age[i].route_cost[k] = 0;
     }
   } 

  printf("��ʍs�����s��:");
  scanf("%d",&x);
     
  //���s�񐔃��[�v
  for(h=0;h<x;h++){
  
    //�ēx�������K�v�ȕ�������
    reset_agent(age, hom);
    
    //���ԏ�̉��z�e�ʐݒ�
    for(i=0;i<NUM_PARKING;i++){
      park[i].capacity = 4;
    }
    
    printf("-------------%d����--------------\n",h);
    
     //�G�[�W�F���g���ׂĈړ��I���܂Ń��[�v
    MoveToDestination(age,unit,com,park,lot,next);

    //�ړ����R�X�g�̎Z�o
    for(k=0;k<NUM_AGENT;k++){
      money(age, park, lot, s_lot, k);    
      cost(age, k);
    }
    
    //���Ԑ����O�Ȓ��ԏ�|�Y.�|�Y����capacity=0
    for(i=0;i<NUM_PARKING;i++){
      if(park[i].num_park == 0)
        park[i].capacity = 0;
    }
    
    //�G�[�W�F���g�̃g���b�v�I��
    if(h != (x-1))
      select_rute(age);
  
     //10�G�[�W�F���g��1�����Ƃ̃g���b�v�I��ω�
    for(i=0;i<10;i++){
      fprintf(fp4,"%d %d  ",i,age[i].t.route_num+1);
     }
    fprintf(fp4,"\n");

  }

  for(i=0;i<NUM_ROUTE_TYPE;i++){
    count[i]=0;
  }
  
  for(i=0;i<NUM_AGENT;i++){
   // printf("%d  x=%f y=%f time %f tired %f route = %d cost = %f\n",i, age[i].x[0], age[i].x[1], age[i].time, age[i].fatigue, age[i].t.route_num, age[i].route_cost[age[i].t.route_num]);
    count[age[i].t.route_num]++;    
  }

  //�e�g���b�v���Ƃ̃G�[�W�F���g�����o��
  for(i=0;i<NUM_ROUTE_TYPE;i++){
    fprintf(fp2,"%d %d\n",i+1, count[i]);
  }
	*/
  return 0;
}



//�G�[�W�F���g������
void setup_agent(struct agent a[], struct home h[]){
  int x, y, z;
  int p, q;               
  int i, j, k;               //���[�v

  for(i=0;i<NUM_AGENT;i++){
    a[i].x[0] = h[i].x[0];
    a[i].x[1] = h[i].x[1];

    a[i].time = 0;
    a[i].money = 0;
    a[i].fatigue = 0;
    a[i].distance_c = 0;
    a[i].distance_w = 0;
    a[i].distance_b = 0;
    a[i].distance_t = 0;
    a[i].parking = 0;
    a[i].bikelot = 0;

    a[i].company = rand()%NUM_COMPANY;


    //�����ŕΌ���ݒ�
    x = 20 + rand() % 30;  
    y = 20 + rand() % 30;
    z = 100 - x - y;
    
    a[i].W[0] = (double)x/100;
    a[i].W[1] = (double)y/100;
    a[i].W[2] = (double)z/100;

    p = rand() % NUM_ROUTE_TYPE;
  
    q = 0;
    j = 0;

    for(k=0;k<NUM_ROUTE_TYPE;k++)
      a[i].route_cost[k] = 0; 
    //�g���b�v�̊��蓖��
    while(q != 100){
      a[i].t.route_num = p;   
      q = route[p][j];    
      a[i].t.unit[j] = q;
      j++;
    }
   
    a[i].t.unit_state = 0;
  }
  check = 0;
}


//�������
void setup_home(struct home h[]){//(-3600�`-400)

  int i,j,k,l,n;
  double m=-L/2.0;          // ����m=Lh
  double sigma=Lh/5.0;  // �W���΍���=2*Lh/10 
  double r1,r2;
  //double limit,limit1,limit2;
  double x_sum=0,y_sum=0,sum2_x=0,sum2_y=0; 

  //int counter[101][101]={0};
  FILE *fp6;

	//srand((unsigned)time(NULL));       /* �����̏����� */

  for(i=0;i<NUM_HOME;i+=2){

    //x���W
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
		//printf("%lf %lf\n",r1,r2);
    h[i].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2)+m);
    h[i+1].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2)+m);
	
    //y���W
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    h[i].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2));
    h[i+1].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2));

		x_sum+=h[i].x[0]+h[i+1].x[0];//����
		y_sum+=h[i].x[1]+h[i+1].x[1];
		sum2_x=(h[i].x[0]+L/2)*(h[i].x[0]+L/2)+(h[i+1].x[0]+L/2)*(h[i+1].x[0]+L/2);//�W���΍�
		sum2_y=h[i].x[1]*h[i].x[1]+h[i+1].x[1]*h[i+1].x[1];
	}
/*
	for(i=0;i<NUM_HOME;i+=2){
		h[i].x[0] = h[i].x[0] - Lh - L/2;
    h[i+1].x[0] = h[i+1].x[0] - Lh - L/2;
    x_sum+=h[i].x[0]+h[i+1].x[0];//����
  }
*/
  for(i=0;i<NUM_HOME;i++){
		if(h[i].x[0]<-10000 || h[i].x[1]<-10000 || 10000 < h[i].x[0] || 10000 < h[i].x[1]){
      printf("error!!%d %f %f \n",i,h[i].x[0], h[i].x[1]);
		}
  }
  printf("x_average=%lf\ny_average=%lf\n",x_sum/NUM_HOME,y_sum/NUM_HOME);//����
  printf("x_deviation=%lf\ny_debiation=%lf\n",sqrt(sum2_x/NUM_HOME),sqrt(sum2_y/NUM_HOME));//�W���΍�



  //���z�}
  if((fp6 = fopen("home0.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }


	for(i=0;i<NUM_HOME;i++){
		fprintf(fp6,"%d %.0lf %.0lf\n",i,h[i].x[0],h[i].x[1]);

	}
  
}
//���]����(����)������
void reset_home(struct home h[],struct home_temp ht[]){
  
	int i,j,k,l,n;
  double sum_x=0,sum_y=0;
	double sum2_x=0,sum2_y=0;
	double average_x=0,average_y=0;
	double deviation_x=0,deviation_y=0;
	double r1,r2;


  //���݂̕��ρE���U�Z�o
  for(i=0;i<NUM_HOME;i++){
		sum_x+=h[i].x[0];
		sum_y+=h[i].x[1];
  }
  average_x = sum_x/NUM_HOME;
	average_y = sum_y/NUM_HOME;

	for(i=0;i<NUM_HOME;i++){
		sum2_x+=(h[i].x[0]-average_x)*(h[i].x[0]-average_x);
    sum2_y+=(h[i].x[1]-average_y)*(h[i].x[1]-average_y);
	}
  deviation_x = sqrt(sum2_x/NUM_HOME);
	deviation_y = sqrt(sum2_y/NUM_HOME);

  printf("x_average=%lf\ny_average=%lf\n",average_x,average_y);//����
  printf("x_deviation=%lf\ny_debiation=%lf\n\n",deviation_x,deviation_y);//�W���΍�

  sum_x=0;
	sum_y=0;
  sum2_x=0;
	sum2_y=0;

  //���]����(10�ӏ�*100�G�[�W�F���g)����
  for(i=0;i<1000;i+=2){

    //x���W
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
		
    ht[i].x[0] = (int)(deviation_x*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2)+average_x);
    ht[i+1].x[0] = (int)(deviation_x*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2)+average_x);
	
    //y���W
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    ht[i].x[1] = (int)(deviation_y*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2)+average_y);
    ht[i+1].x[1] = (int)(deviation_y*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2)+average_y);

		sum_x+=ht[i].x[0]+ht[i+1].x[0];//����
		sum_y+=ht[i].x[1]+ht[i+1].x[1];
		sum2_x=(ht[i].x[0]-average_x)*(ht[i].x[0]-average_x)+(ht[i+1].x[0]-average_x)*(ht[i+1].x[0]-average_x);//�W���΍�
		sum2_y=(ht[i].x[1]-average_y)*(ht[i].x[1]-average_y)+(ht[i+1].x[1]-average_y)*(ht[i+1].x[1]-average_y);
	}

/*
	for(i=0;i<1000;i+=2){
		ht[i].x[0] = ht[i].x[0] - Lh - L/2;
    ht[i+1].x[0] = ht[i+1].x[0] - Lh - L/2;
    sum_x+=ht[i].x[0]+ht[i+1].x[0];//����
  }
*/
	
/*
	for(i=0;i<1000;i++){
		printf("%d %.lf %.lf\n",i,ht[i].x[0],ht[i].x[1]);
	}
	*/
/*
  printf("x_average=%lf\ny_average=%lf\n",sum_x/1000,sum_y/1000);//����
  printf("x_deviation=%lf\ny_debiation=%lf\n",sqrt(sum2_x/1000),sqrt(sum2_y/1000));//�W���΍�
*/
}


//�n��v�Z
void home_price(struct home h[],struct home_temp ht[],struct company c[]){

  int x,y,i,j,k;
	double A=30000*140*100;//�n��W����*�P���іʐ�50(*100��s�����Ǝ��ې��̍��~�j(����Ɉړ���{*1000or��s�����ƍ��̐ݒ�ʐς̍�)
	double H=100;       //��Вn��̏d��=2

 
  for(j=0;j<(Yh-Yl)/100;j++){
	  for(k=0;k<(Xh-Xl)/100;k++){
      home_dencity[j][k]=0;
			com_dencity[j][k]=0;
	  }
  }

	//home�̖��x�v�Z
  for(i=0;i<NUM_HOME;i++){
    for(j=0,y=Yl ; y<=Yh ; j++,y+=100){
			if(y<=h[i].x[1] && h[i].x[1]<y+100){
	      for(k=0,x=Xl ; x<=Xh ; k++,x+=100){			
			    if(x<=h[i].x[0] && h[i].x[0]<x+100){
				    home_dencity[j][k]++;
					  //printf("%d %d h%d \n",j,k,home_dencity[j][k]);			
	        }
			  }
			}
		}
	}

	//com�̖��x�v�Z
  for(i=0;i<NUM_COMPANY;i++){
    for(j=0,y=Yl ; y<=Yh ; j++,y+=100){
			if(y<=c[i].x[1] && c[i].x[1]<y+100){
			//printf("%d %d com%d \n",j,k,com_dencity[j][k]);			
	      for(k=0,x=Xl ; x<=Xh ; k++,x+=100){			
			    if(x<=c[i].x[0] && c[i].x[0]<x+100){
				    com_dencity[j][k]++;
					  //printf("%d %d com%d \n",j,k,com_dencity[j][k]);			
	        }
			  }
			}
		}
	}


	//�]�����̒n��Z�o
	for(i=0;i<1000;i++){                         //i= 100�G�[�W�F���g*10�ӏ�
    for(j=0,y=Yl ; y<=Yh ; j++,y+=100){
			if(y<=ht[i].x[1] && ht[i].x[1]<y+100){
	      for(k=0,x=Xl ; x<=Xh ; k++,x+=100){			
			    if(x<=ht[i].x[0] && ht[i].x[0]<x+100){
					  ht[i].price = A*(home_dencity[j][k]+H*com_dencity[j][k])/10000;//�n��(���]����)
					  //fprintf(fp8,"%d %lf \n",i,ht[i].price);			
	        }
			  }
		  }
	  }
	}
	
	//����ʒu�̒n��Z�o
	for(i=0;i<NUM_HOME;i++){                         
    for(j=0,y=Yl ; y<=Yh ; j++,y+=100){
			if(y<=h[i].x[1] && h[i].x[1]<y+100){
	      for(k=0,x=Xl ; x<=Xh ; k++,x+=100){			
			    if(x<=h[i].x[0] && h[i].x[0]<x+100){
					  h[i].price = A*(home_dencity[j][k]+H*com_dencity[j][k])/10000;//�n��(����)
					  //fprintf(fp8,"%d %lf \n",i,ht[i].price);			
	        }
			  }
		  }
	  }
	}

}

//�]����I��
void select_home(struct home h[],struct home_temp ht[],int counter,FILE *fp8){

	int i,j;
  double best=99999999; 
  int best_num;

	for(i=counter;i<counter+100;i++){//h[]:�G�[�W�F���g�ԍ�,ht[]:��̈ʁE�\�̈�
		best=99999999;
		for(j=0;j<1000;j+=100){//�S�̈�
		  if(ht[(i-counter)+j].price<best){
	      h[i].x[0]=ht[(i-counter)+j].x[0];
        h[i].x[1]=ht[(i-counter)+j].x[1];
				 best=ht[(i-counter)+j].price;
				 best_num=(i-counter)+j;
		  }
	  }
		fprintf(fp8,"%d %.0lf %d\n",i,best,best_num);//�G�[�W�F���g�ԍ��@�]���R�X�g
	}
  //fprintf(fp8,"\ncycle\n");
}

//���r���e�B�}�l�W�����g
void mobility_management(struct agent a[]){

  int i,j;
  int counter=0;
  int q;

	for(i=0;i<NUM_AGENT;i++){
		if(a[i].t.route_num==3 || a[i].t.route_num==5){
			counter++;
		}
	}
	
	//��%�������ԗ��p����d�ԗ��p��
	counter *= MM;

	for(i=0;i<NUM_AGENT;i++){
		if(a[i].t.route_num==3 || a[i].t.route_num==5){
			if(counter > 0){
			  counter--;
		    a[i].t.route_num = 0;
       j=0;
    		q=0;
    
       while(q != 100){     
      	  q = route[a[i].t.route_num][j];    
         a[i].t.unit[j] = q;      
      	  j++;
        }  
			}
		}
	}

}

//���(�ړI�n)������
void setup_company(struct company c[]){
  int i,j,k,l,n;
  double m=L/2;          // ����m=Lb
  double sigma=Lb/5.0;  // �W���΍���=2*Lb/10 
  double r1,r2;
  double limit;
  double x_sum=0,y_sum=0,sum2_x=0,sum2_y=0; 

  
  FILE *fp5;
  

  for(i=0;i<NUM_COMPANY;i+=2){

    //x���W
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    c[i].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2)+m);
    c[i+1].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2)+m);
	
    //y���W
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    c[i].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2));
    c[i+1].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2));

		x_sum+=c[i].x[0]+c[i+1].x[0];//����
    y_sum+=c[i].x[1]+c[i+1].x[1];
	  sum2_x=(c[i].x[0]+2000)*(c[i].x[0]+2000)+(c[i+1].x[0]+2000)*(c[i+1].x[0]+2000);//�W���΍�
	  sum2_y=c[i].x[1]*c[i].x[1]+c[i+1].x[1]*c[i+1].x[1];
  }
/*
  for(i=0;i<NUM_COMPANY;i+=2){
    c[i].x[0] = c[i].x[0] - Lb + L/2;
    c[i+1].x[0] = c[i+1].x[0] - Lb + L/2;
    x_sum+=c[i].x[0]+c[i+1].x[0];//����
  }
	*/

/*
  for(i=0;i<NUM_COMPANY;i++){
    printf("%d %f %f \n",i,c[i].x[0], c[i].x[1]);
  }
  printf("x_average=%lf\ny_average=%lf\n",x_sum/NUM_COMPANY,y_sum/NUM_COMPANY);//����
  printf("x_deviation=%lf\ny_debiation=%lf\n",sqrt(sum2_x/NUM_COMPANY),sqrt(sum2_y/NUM_COMPANY));//�W���΍�
*/

  //���z�}
  if((fp5 = fopen("business.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }
  
	for(i=0;i<NUM_COMPANY;i++){
	  fprintf(fp5,"%d %.0lf %.0lf\n",i,c[i].x[0],c[i].x[1]);
	}

}  

//���֏�,���ԏꏉ����
void setup_park_lot(struct parking p[], struct bikelot l[], struct stationlot sl){

  int i,j,k,o,n;
  double m=L/2;          // ����m=Lb
  double sigma=Lb/5.0;  // �W���΍���=2*Lb/10 
  double r1,r2;
  double limit;
  double x_sum=0,y_sum=0,sum2_x=0,sum2_y=0; 



  for(i=0;i<NUM_PARKING;i+=2){

    //x���W
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    p[i].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2)+m);
    p[i+1].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2)+m);
	
    //y���W
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    p[i].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2));
    p[i+1].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2));
    
		x_sum+=p[i].x[0]+p[i+1].x[0];//����
    y_sum+=p[i].x[1]+p[i+1].x[1];
	  sum2_x=(p[i].x[0]+2000)*(p[i].x[0]+2000)+(p[i+1].x[0]+2000)*(p[i+1].x[0]+2000);//�W���΍�
	  sum2_y=p[i].x[1]*p[i].x[1]+p[i+1].x[1]*p[i+1].x[1];

	  x_sum+=l[i].x[0]+l[i+1].x[0];//����
    y_sum+=l[i].x[1]+l[i+1].x[1];//����
	  sum2_x=(l[i].x[0]+2000)*(l[i].x[0]+2000)+(l[i+1].x[0]+2000)*(l[i+1].x[0]+2000);//�W���΍�
	  sum2_y=l[i].x[1]*l[i].x[1]+l[i+1].x[1]*l[i+1].x[1];
  }


  for(i=0;i<NUM_BIKELOT;i+=2){

    //x���W
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    l[i].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2)+m);
    l[i+1].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2)+m);
	
    //y���W
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    l[i].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2));
    l[i+1].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2));

	  x_sum+=l[i].x[0]+l[i+1].x[0];//����
    y_sum+=l[i].x[1]+l[i+1].x[1];//����
	  sum2_x=(l[i].x[0]+2000)*(l[i].x[0]+2000)+(l[i+1].x[0]+2000)*(l[i+1].x[0]+2000);//�W���΍�
	  sum2_y=l[i].x[1]*l[i].x[1]+l[i+1].x[1]*l[i+1].x[1];
  }

  for(i=0;i<NUM_BIKELOT;i+=2){
    l[i].x[0] = l[i].x[0] - Lb + L/2;
    l[i+1].x[0] = l[i+1].x[0] - Lb + L/2;
    x_sum+=l[i].x[0]+l[i+1].x[0];//����
  }


  for(i=0;i<NUM_PARKING;i++){
 
    p[i].capacity = 4;               //�e�ʐݒ�    
    p[i].charge = 800;
    p[i].num_park = 0;
  }

  for(i=0;i<NUM_BIKELOT;i++){    

    l[i].capacity = 4;             //�e�ʐݒ�
    l[i].charge = 150;
    l[i].num_lot = 0;
  }
  
  sl.capacity = 4; 
  sl.charge = 150; 
  sl.num_lot =0;
  
}

//�g���b�v�̓���
void setup_trip(struct unit_trip u[]){
  int i;
 
  u[0].method = WALK;   u[0].goal_id = STATION_A;  
  u[1].method = WALK;   u[1].goal_id = COMPANY;  
  u[2].method = BIKE;   u[2].goal_id = STATION_A;  
  u[3].method = BIKE;   u[3].goal_id = COMPANY;    
  u[4].method = BIKE;   u[4].goal_id = BIKELOT;   
  u[5].method = CAR;    u[5].goal_id = COMPANY;
  u[6].method = CAR;    u[6].goal_id = HIGHWAY_S;   
  u[7].method = CAR;    u[7].goal_id = PARKING;
  u[8].method = CAR_H;  u[8].goal_id = HIGHWAY_E; 
  u[9].method = TRAIN;  u[9].goal_id = STATION_B;
  u[10].method = BIKE;  u[10].goal_id = STATION_LOT;
  
 
  route[0][0] = W_STATION_A;   route[0][1] = T_STATION_B;  route[0][2] = W_COMPANY;  route[0][3]= END;
  route[1][0] = B_STATION_LOT; route[1][1] = T_STATION_B;  route[1][2] = W_COMPANY;  route[1][3]= END;
  route[2][0] = W_COMPANY;     route[2][1] = END;
  route[3][0] = C_PARKING;     route[3][1] = W_COMPANY;    route[3][2] = END;
  route[4][0] = B_BIKELOT;     route[4][1] = W_COMPANY;    route[4][2] = END; 
  route[5][0] = C_HIGHWAY_S;   route[5][1] = C_HIGHWAY_E;  route[5][2] = C_PARKING;  route[5][3] = W_COMPANY;  route[5][4]= END;

  //��@����
  route[6][0] = B_COMPANY;    route[6][1] = END;
  route[7][0] = B_STATION_A;  route[7][1] = T_STATION_B;  route[7][2] = W_COMPANY;  route[7][3]= END;
}

//�G�[�W�F���g�ď�����
void reset_agent(struct agent a[],struct home h[]){
  int i;               //���[�v
  
  for(i=0;i<NUM_AGENT;i++){
    a[i].x[0] = h[i].x[0];
    a[i].x[1] = h[i].x[1];

    a[i].time = 0;
    a[i].money = 0;
    a[i].fatigue = 0;
    a[i].distance_c = 0;
    a[i].distance_w = 0;
    a[i].distance_b = 0;
    a[i].distance_t = 0;
    a[i].parking = 0;
    a[i].bikelot = 0;

		a[i].t.unit_state = 0;

  }
  check = 0;
  //count_CAR_H = 0;
  //count_CAR = 0;  
}

//�G�[�W�F���g�����ׂĈړ��I������܂Ń��[�v
void MoveToDestination(struct agent a[],struct unit_trip u[],struct company c[],struct parking p[],struct bikelot bl[],double next[]){

  int i;

  while(check < NUM_AGENT){
    //�Ԗ��x�̌v�Z
	  calc_dencity(a, u);
			
    for(i=0;i<NUM_AGENT;i++){    
	     //�ړ��I�����ۂ�
	    if(a[i].t.unit[a[i].t.unit_state] != 100){
	       //�G�[�W�F���g�̑��x�̍X�V  
	      a[i].v = speed(u[a[i].t.unit[a[i].t.unit_state]].method);
	        
				 //�a�ؑ��x�E���̉��Z			  
				 congestion(a, u, i);
					
	       //���̖ڕW�n�_��ݒ�
	      next_spot(a, c, p, bl, u, next, i);
	       //�ړ�
	      movement(a, next, u, i);
	        		
	       //���̌v�Z
	      tired(a ,u, i);
	       //�ڕW�n�_�֓������ۂ�
	      if(check_arrival(a, next, i)){
	         //�O�̖ړI�n�����ԏ�Ȃ�   
          if(a[i].t.unit[a[i].t.unit_state-1] == C_PARKING){
              //���ԏꉼ�z�e�ʂ����炷           
            p[a[i].parking].num_park++;   
           }
         }
 			 }  
		 }
  }
}

//�g���b�v�̊��蓖��
void select_rute(struct agent a[]){
  
  int i, h, q, j;
  int best;
  
	//srand((unsigned)time(NULL));       /* �����̏����� */

  for(i=0;i<NUM_AGENT;i++){
    if((double)rand()/RAND_MAX < E){
      a[i].t.route_num = rand() % NUM_ROUTE_TYPE;
    }

    else{
      for(h=0; h<NUM_ROUTE_TYPE; h++){
	      if(a[i].route_cost[h]  > 1){
	        best = h;
	      }
      }
     
      for(h=0; h<NUM_ROUTE_TYPE; h++){
	      if(a[i].route_cost[h] > 1){
					if(a[i].route_cost[h] < a[i].route_cost[best]){
	          best = h;
					}
	      }
      }
      a[i].t.route_num = best;
    }


    j=0;
    q=0;
    
    while(q != 100){     
      q = route[a[i].t.route_num][j];    
      a[i].t.unit[j] = q;      
      j++;
    }
    a[i].t.unit_state = 0;

  }


  E *= G; 

}

//��ʋ@�ւ̑��x
double speed(int trip){

  if(trip == WALK)
    return Vw;

  if(trip == BIKE)
    return Vb;

  if(trip == CAR)
    return Vc;

  if(trip == CAR_H)
    return Vch;

  if(trip == TRAIN)
    return Vt;

  return 0;//�ǂ�����Ă͂܂�Ȃ������Ƃ�
}

//���̖ڕW�n�_�̊��蓖��
void next_spot(struct agent a[], struct company c[], struct parking par[], struct bikelot bl[], struct unit_trip u[], double next[], int i)
{ 
 
  int num;

  if(u[a[i].t.unit[a[i].t.unit_state]].goal_id == STATION_A || u[a[i].t.unit[a[i].t.unit_state]].goal_id == STATION_LOT){
    next[0] = station_a[0];   
    next[1] = station_a[1];
  }

  if(u[a[i].t.unit[a[i].t.unit_state]].goal_id == STATION_B){
    next[0] = station_b[0];   
    next[1] = station_b[1];
  }

  if(u[a[i].t.unit[a[i].t.unit_state]].goal_id == COMPANY){
	next[0] = c[a[i].company].x[0];
	next[1] = c[a[i].company].x[1];
  }

  if(u[a[i].t.unit[a[i].t.unit_state]].goal_id == HIGHWAY_S){
    next[0] = a[i].x[0];   
    next[1] = 10;

  }

  if(u[a[i].t.unit[a[i].t.unit_state]].goal_id == HIGHWAY_E){//�������H��̒��ԏ�x���W�܂ňړ�.���p���钓�ԏ�͂����Ō��߂Ă���
    if(u[a[i].t.unit[a[i].t.unit_state]].method == CAR_H){
      num = selectparking(c, a, par, i); 

      next[0] = par[num].x[0];   
      next[1] = 10;
    }
  }

  if(u[a[i].t.unit[a[i].t.unit_state]].goal_id == BIKELOT){
    num=selectbiklot(c, a, bl, i);
    next[0] = bl[num].x[0];   
    next[1] = bl[num].x[1];
  }
  
  if(u[a[i].t.unit[a[i].t.unit_state]].goal_id == PARKING){
    num = selectparking(c, a, par,i); 
    next[0] = par[num].x[0];       
    next[1] = par[num].x[1];
  }
}

//�ړ�
void movement(struct agent a[], double next[], struct unit_trip u[], int i){

  double vec_x, vec_y;
  double compare_x, compare_y;


  if(a[i].t.unit[a[i].t.unit_state] != 100){
    a[i].time ++;
    vec_x = next[0] - a[i].x[0]; 
    vec_y = next[1] - a[i].x[1];

    if(sqrt(vec_x * vec_x + vec_y * vec_y)==0){
      a[i].x[0] = next[0]; 
      a[i].x[1] = next[1];
    }
    else{
      a[i].x[0] = a[i].x[0] + a[i].v * (vec_x / sqrt(vec_x * vec_x + vec_y * vec_y)); 
      a[i].x[1] = a[i].x[1] + a[i].v * (vec_y / sqrt(vec_x * vec_x + vec_y * vec_y));
    
      compare_x = next[0] - a[i].x[0];
      compare_y = next[1] - a[i].x[1];
      
      if(u[a[i].t.unit[a[i].t.unit_state]].method == CAR || u[a[i].t.unit[a[i].t.unit_state]].method == CAR_H){
	      a[i].distance_c += a[i].v;
      }
      if(u[a[i].t.unit[a[i].t.unit_state]].method == WALK){
         a[i].distance_w += a[i].v;
      }
      if(u[a[i].t.unit[a[i].t.unit_state]].method == BIKE){
         a[i].distance_b += a[i].v;
      }
      if(u[a[i].t.unit[a[i].t.unit_state]].method == TRAIN){
         a[i].distance_t += a[i].v;
      }
      
      if((vec_x * compare_x) < 0 || (vec_y * compare_y) < 0){
				a[i].x[0] = next[0]; 
				a[i].x[1] = next[1];
      }
    }
  }
	/*
  if(u[a[i].t.unit[a[i].t.unit_state]].method == CAR_H){//�������g���ĕ��S���̌v�Z?.
    count_CAR_H++;
  }
  if(u[a[i].t.unit[a[i].t.unit_state]].method == CAR){
    count_CAR++;
  }
	*/
} 

//�����m�F//��������1��Ԃ��悤�ɕύX
int check_arrival(struct agent a[], double next[], int i){

  if(a[i].t.unit[a[i].t.unit_state] != 100){
    if(a[i].x[0]==next[0] && a[i].x[1] == next[1]){

      a[i].t.unit_state++;

      if(a[i].t.unit[a[i].t.unit_state] == 100){
       check ++;
      }

      return 1;
    }
  }
  
  return 0;
}

//��ʋ@�ւɂ����v�Z
void tired(struct agent a[], struct unit_trip u[], int i){

  if(u[a[i].t.unit[a[i].t.unit_state]].method == WALK)
    a[i].fatigue += 11;

  if(u[a[i].t.unit[a[i].t.unit_state]].method == BIKE)
    a[i].fatigue += 4;

  if(u[a[i].t.unit[a[i].t.unit_state]].method == CAR || u[a[i].t.unit[a[i].t.unit_state]].method == CAR_H)
    a[i].fatigue += 1; 
  
  if(u[a[i].t.unit[a[i].t.unit_state]].method == TRAIN)
    a[i].fatigue += 2;
}    

//�g���b�v�ɂ����������z
void money(struct agent a[],struct parking p[], struct bikelot l[], struct stationlot sl, int i){

	
  if(a[i].t.route_num == 3 || a[i].t.route_num == 5)
    a[i].money += p[0].charge; 

  if(a[i].t.route_num == 4)
    a[i].money += l[0].charge; 

  if(a[i].t.route_num == 0 || a[i].t.route_num == 1 || a[i].t.route_num==7)
   a[i].money += charge_t; 

  if(a[i].t.route_num == 1)
    a[i].money += sl.charge;

  if(a[i].t.route_num == 6 || a[i].t.route_num == 7){
    if((double)rand()/RAND_MAX < P_remove){
      a[i].money += charge_rm;
    }
  }

  a[i].money += a[i].distance_c * charge_g;
  
}

//�R�X�g�Z�o
void cost(struct agent a[], int i){
  double money, fatigue, time;

  if(a[i].t.route_num == 0 || a[i].t.route_num == 1 || a[i].t.route_num == 7)
    a[i].time += 10;

  time = a[i].time * change_t;
  money = a[i].money;
  fatigue = a[i].fatigue * change_f; 
  
  //printf("%d time%.0lf money%.0lf fatigue%.0lf\n",i,a[i].time*change_t,a[i].money,a[i].fatigue*change_f);  

  a[i].route_cost[a[i].t.route_num] = (1-B)*a[i].route_cost[a[i].t.route_num] + B*(fatigue + time + money);

}

//���]����̈ړ��R�X�g�Z�o
double cost_home(struct agent a[],int i){
  double money, fatigue, time;

  if(a[i].t.route_num == 0 || a[i].t.route_num == 1 || a[i].t.route_num == 7)
    a[i].time += 10;

  time = a[i].time * change_t;
  money = a[i].money;
  fatigue = a[i].fatigue * change_f;
  
  //printf("%d time%.0lf money%.0lf fatigue%.0lf\n",i,a[i].time*change_t,a[i].money,a[i].fatigue*change_f);     

  return(fatigue + time + money);
  //printf("%d");
}

//�Ԗ��x�v�Z
void calc_dencity(struct agent a[],struct unit_trip u[]){
  
  int x,y,i,j,k;

  for(j=0;j<(Yh-Yl)/100;j++){ 
	  for(k=0;k<(Xh-Xl)/100;k++){
      car_dencity[j][k]=0;
	  }
  }

  for(i=0;i<NUM_AGENT;i++){
		if(a[i].t.unit[a[i].t.unit_state] != 100){
	    if(u[a[i].t.unit[a[i].t.unit_state]].method == CAR_H || u[a[i].t.unit[a[i].t.unit_state]].method == CAR){
        for(j=0,y=Yl ; y<=Yh ; j++,y+=100){
		      if(y<=a[i].x[1] && a[i].x[1]<y+100){
	          for(k=0,x=Xl ; x<=Xh ; k++,x+=100){			
					    if(x<=a[i].x[0] && a[i].x[0]<x+100){
						    car_dencity[j][k]++;
						    a[i].d_num[0]=j;//car��car_h�̂ݏ������D
						    a[i].d_num[1]=k;
						    //printf("%d %d %d\n",i,a[i].d_num[0],a[i].d_num[1]);
						    //printf("%d %d %d \n",j,k,car_dencity[j][k]);
					    }
	          }
			    }
		    }
	    }
		}
	}

}

//�a�؂ɂ�鑬�x�ω��E�����Z
void congestion(struct agent a[], struct unit_trip u[], int i){//�ǉ��K�v 
	  
	  double temp;


//printf("%d %d %d\n",i,a[i].d_num[0],a[i].d_num[1]);
		temp=a[i].fatigue;
	  if(u[a[i].t.unit[a[i].t.unit_state]].method == CAR_H || u[a[i].t.unit[a[i].t.unit_state]].method == CAR){
			//printf("%.1lf\n",(-1.14*(car_dencity[a[i].d_num[0]][a[i].d_num[1]]-25)+80)*1000/60);
		  if((-1.14*(car_dencity[a[i].d_num[0]][a[i].d_num[1]]-25)+80)*1000/60 < a[i].v){//�Z�O�����g�G���[a[i].d_num�����������D
				//printf("%.1lfm/min\n",(-1.14*(car_dencity[a[i].d_num[0]][a[i].d_num[1]]-25)+80)*1000/60);
				a[i].v=(-1.14*(car_dencity[a[i].d_num[0]][a[i].d_num[1]]-25)+80)*1000/60;//���x
				a[i].fatigue += car_dencity[a[i].d_num[0]][a[i].d_num[1]]*0.1;//���
      
				if(a[i].v <= 0){
					a[i].v=83;//�Œᑬ�x5km/h 83m/��
					a[i].fatigue += 10-car_dencity[a[i].d_num[0]][a[i].d_num[1]]*0.1;//�ő���10
				}
				//printf("%d %d %d  %.1lfkm/h ���%.1lf\n\n",i,a[i].d_num[0],a[i].d_num[1],a[i].v*60/1000,a[i].fatigue-temp);
			}
		}

}

//���֏�̑I��
int selectbiklot(struct company c[],struct agent a[], struct bikelot bl[], int i){
  int h, l,lx, ly, best_num ;
  double  best=99999999;

  for(h=0;h<NUM_BIKELOT;h++){
    lx = c[a[i].company].x[0] - bl[h].x[0];
    ly = c[a[i].company].x[1] - bl[h].x[1];
    l=lx*lx +ly*ly;    
   
    if(l<best){
      best_num=h;
      best = l;  
    }
  }
  a[i].bikelot=best_num;

  return best_num;
}

//���ԏ�̑I��
int selectparking(struct company c[],struct agent a[], struct parking p[], int i){
  int h, l,lx, ly, best_num;
  double  best=99999999;
  double temp1,temp3;

  for(h=0;h<NUM_PARKING;h++){
    if(p[h].capacity != 0){//���ԏꂪ�|�Y���ĂȂ����
      lx = c[a[i].company].x[0]-p[h].x[0];
      ly = c[a[i].company].x[1]-p[h].x[1];
      l=lx*lx +ly*ly;    

      if(l<best){  
        best_num=h;
        best = l;
      }
    }
  }
  a[i].parking=best_num;

  return best_num;
}
