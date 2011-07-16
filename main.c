#include<stdio.h>
#define _USE_MATH_DEFINES//VisualStudio用
#include<math.h> 
#include<time.h>
#include<stdlib.h>

enum methods {WALK, BIKE, CAR, CAR_H, TRAIN};
enum goals {STATION_A, STATION_B, COMPANY, HIGHWAY_S, HIGHWAY_E, PARKING, BIKELOT, STATION_LOT};
enum routs {W_STATION_A, W_COMPANY, B_STATION_A, B_COMPANY, B_BIKELOT, C_COMPANY, C_HIGHWAY_S, C_PARKING, C_HIGHWAY_E, T_STATION_B, B_STATION_LOT,END = 100};

enum methods method;
enum goals goal;
enum routs routs;
 
#define NUM_AGENT 1000                         //エージェント数 
#define NUM_HOME  1000                       //家の数
#define NUM_COMPANY 1000                        //会社の数
#define NUM_PARKING 1000                          //駐車場の数(初期値)
#define NUM_BIKELOT 1000                          //駐輪場の数
#define NUM_ROUTE_TYPE 8                          //トリップの種類数
#define MAX_ROUTE_DEPTH 10                          //手段トリップの最大数(実際に使ってるのは4つまで)

#define Xl -6000                       //仮想空間のX軸最低値-6000
#define Xh 6000                        //　　　　　　 最大値6000
#define Yl -4000                       //仮想空間のY軸最低値-4000
#define Yh 4000                        //　　　　　 　最大値4000

/////////////////////////////////////////////////////////////////////////////
#define MM 0                           //モビリティマネジメントの割合(0-1)
double charge_g = 0.1; //0.01 ;                  //ガソリン代(円/m)
/////////////////////////////////////////////////////////////////////////////

double L = 4000;                         //２駅間の距離(m)
double Vw = 66;                          //歩く速度(m/分)(4km/h)
double Vb = 267;//160;                         //自転車の速度(m/分)(9.6km/h)
double Vc = 450;                         //自動車の速度(m/分)(27km/h)
double Vch = 833;                        //自動車の幹線道路の速度(m/分)(50km/h)
double Vt;                               //電車の速度
double Lb= 3200;                         //businesszoneの半径 
double Lh= 3200;                         //homezoneの半径
int route[NUM_ROUTE_TYPE][MAX_ROUTE_DEPTH];    //ルート

int car_dencity[(Yh-Yl)/100][(Xh-Xl)/100];              //区画ごとの車密度
int home_dencity[(Yh-Yl)/100][(Xh-Xl)/100];             //区画ごとの自宅密度
int com_dencity[(Yh-Yl)/100][(Xh-Xl)/100];              //区画ごとの会社密度

double E = 0.7;                          //変異確率の初期値
double G = 0.94;                         //変異確率に掛ける定数
double B = 0.5;                          //学習係数

double P_remove = 0.5;                   //自転車撤去確率
double charge_rm = 3000;                 //撤去返還代
double charge_t;                          //電車代
  
//double count_CAR_H = 0;
//double count_CAR = 0;
int    count_par =0;                     //倒産した駐車場の数

//変換変数
double change_f = 20;                   //疲れ
double change_t = 30;                   //時間

int route[NUM_ROUTE_TYPE][MAX_ROUTE_DEPTH];    //ルート 
int check;                               //エージェント到着確認のための変数


  //移動手段と目的地
struct unit_trip{
  int method;                           //方法
  int goal_id;                          //現在の目的地

};
                                      
struct trip{
  int unit[10];                         //トリップの[]番目の手段トリップ       
  int unit_state;                       //エージェントは選択したトリップのunit_state番目の手段トリップを使っている
  int route_num;                        //エージェントが選択したトリップ番号
};


//エージェント
struct agent{
  double x[2];                     //エージェントの位置 
  double W[3];                     //エージェントの偏向(しんどさ,金,時間)
  double route_cost[NUM_ROUTE_TYPE]; //各トリップにかかるコスト
  struct trip t;                   //トリップ      
  double v;                        //現在の速度
  double time;                     //移動時間
  double distance_c;               //移動距離(車)
  double distance_w;               //移動距離(徒歩)
  double distance_b;               //移動距離(自転車)
  double distance_t;               //移動距離(電車)
  double money;                    //費用
  double fatigue;                  //疲れ
  int company;                     //目的地ナンバー
  int parking;                     //利用している駐車場のナンバー
  int bikelot;                     //　　　　　　駐輪場のナンバー

	int d_num[2];                    //現在地ゾーンナンバー(エージェントの今いる区画)
};


//住宅
struct home{
  double x[2];                     //家の座標
  double price;                    //地代
};

//仮転居先
struct home_temp{
	double x[2];                      //仮転居先の座標
	double price;                     //地代
	//double cost;
};

//会社
struct company{
  double x[2];                     //会社の座標 
};

//区画ごとの密度
struct density{                   
  double car;                      //車密度
	double home;                     //自宅密度
	double com;                      //会社密度
	double Hprice;                   //区画ごとの地代
};

//駐車場
struct parking{
  double x[2];                    //駐車場の座標 
  int capacity;                    //容量
  int charge;                      //駐車代
  int num_park;                    //駐車台数
};

//駐輪場
struct bikelot{   
  double x[2];                    //駐輪場の座標
  int capacity;                    //容量
  int charge;                      //駐輪代
  int num_lot;                     //駐輪台数
};                                   
//駅の駐輪場
struct stationlot{
 double x[2];                    //駅駐輪場の座標
  int capacity;                  //容量
  int charge;                    //駐輪代
  int num_lot;                   //駐輪台数
};  

//駅
double station_a[2];             //駅Aの座標
double station_b[2];             //駅Bの座標

//struct density den[500][500];

//関数 
//エージェント,住宅,会社などの初期設定
//初期自宅座標初期化
void setup_home(struct home h[]);
//仮転居先(自宅)初期化
void reset_home(struct home h[],struct home_temp ht[]);
//地代計算
void home_price(struct home h[],struct home_temp ht[],struct company c[]);
//転居先決定
void select_home(struct home h[],struct home_temp ht[],int counter,FILE *fp8);

//モビリティマネジメント
void mobility_management(struct agent a[]);

//会社座標初期化
void setup_company(struct company c[]);
//各トリップ内の手段トリップ初期化
void setup_trip(struct unit_trip u[]);
//エージェントの初期化
void setup_agent(struct agent a[], struct home h[]);
//駐車場・駐輪場座標初期化
void setup_park_lot(struct parking p[], struct bikelot l[], struct stationlot sl);

//エージェントすべて移動終了までループ
void MoveToDestination(struct agent a[],struct unit_trip u[],struct company c[],struct parking p[],struct bikelot bl[],double next[]);

//エージェントのルート選択
void select_rute(struct agent a[]);                             
//double select_trip();   //移動手段の決定                                                                                                                     
//エージェントの再初期化
void reset_agent(struct agent a[], struct home h[]);
// エージェントの現在の速度の計算                                      
double speed(int trip);                                   
//エージェントの移動                           
void movement(struct agent a[],double next[], struct unit_trip u[], int i);                                                            
//次の目的地の決定
void next_spot(struct agent a[], struct company c[], struct parking par[], struct bikelot bl[], struct unit_trip u[], double next[], int i);       
//到着確認
int check_arrival(struct agent a[], double next[], int i);                                                         
//移動にかかった疲れ算出
void tired(struct agent a[], struct unit_trip u[], int i);
//移動にかかった料金算出    
void money(struct agent a[], struct parking p[], struct bikelot l[], struct stationlot sl, int i);                
//移動総コスト算出
void cost(struct agent a[], int i);
//仮転居先での移動総コスト算出
double cost_home(struct agent a[], int i);
//車密度の計算
void calc_dencity(struct agent a[], struct unit_trip u[]);
//渋滞による速度減少・疲れの加算
void congestion(struct agent a[], struct unit_trip u[], int i);
//駐輪場選択
int selectbiklot(struct company c[], struct agent a[], struct bikelot bl[], int i);
//駐車場選択
int selectparking(struct company c[], struct agent a[], struct parking p[], int i);


main()
{
	//宣言初期化  
  struct unit_trip unit[20];                        //トリップ  
  struct agent age[NUM_AGENT];                              //エージェント
  struct home hom[NUM_HOME];                               //ホーム
	struct home_temp homt[1000];                              //仮転居先
  struct company com[NUM_COMPANY];                            //目的地
  struct parking park[NUM_PARKING];                           //駐輪場
  struct bikelot lot[NUM_BIKELOT];                            //駐車場
  struct stationlot s_lot;                                    //駅駐輪場

  int i , h,j,k,l,n,q;                                     //ループ
  int counter;
  int x;                                                   //試行回数
  
  double next[2];                                          //エージェントの現在の目標座標
  int count[NUM_ROUTE_TYPE] = {0};                           //各トリップを選択したエージェント数
	
	//int counter2[101][101]={0};
	//int counter_m = 0;

	//int car_dencity[][1000]={0};//車密度[Lh*2/100][L+Lh+Lb]


  //int distance_a[NUM_ROUTE_TYPE] = {0};//平均距離(自宅-駅A)
  //int distance_b[NUM_ROUTE_TYPE] = {0};//平均距離(自宅-目的地)

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

  charge_t = L * 0.04;//電車代
  station_a[0] = -L/2;   station_a[1] = 0;//駅座標
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

	//分布図
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
  
  srand((unsigned)time(NULL));       /* 乱数の初期化 */

  //初期化
  setup_trip(unit);
  setup_home(hom);
  setup_company(com);
  setup_agent(age, hom);
  setup_park_lot(park,lot,s_lot);

//電車速度算出
// L=5000 とすると Vt =1100m/min = 66km/h くらいか．
  Vt = L/7 +350;
  if(Vt>1100)
    Vt=1100;

//交通行動選択

  printf("交通行動試行回数:");
  scanf("%d",&x);
  
  //試行回数ループ
  for(h=0;h<x;h++){
  
    //再度初期化必要な物初期化
    reset_agent(age, hom);
    
    //駐車場の仮想容量設定
    for(i=0;i<NUM_PARKING;i++){
      park[i].capacity = 4;
    }
    
    printf("-------------%d日目--------------\n",h);
    
     //エージェントすべて移動終了までループ
    MoveToDestination(age,unit,com,park,lot,next);

    //移動総コストの算出
    for(k=0;k<NUM_AGENT;k++){
      money(age, park, lot, s_lot, k);    
      cost(age, k);
    }
    
    //駐車数が０な駐車場倒産.倒産時はcapacity=0
    for(i=0;i<NUM_PARKING;i++){
      if(park[i].num_park == 0)
        park[i].capacity = 0;
    }
    
    //エージェントのトリップ選択
    if(h != (x-1))
      select_rute(age);
  
     //10エージェントの1日ごとのトリップ選択変化
    for(i=0;i<10;i++){
      fprintf(fp4,"%d %d  ",i,age[i].t.route_num+1);
     }
    fprintf(fp4,"\n");
    
  }
  
  for(i=0;i<NUM_AGENT;i++){
   // printf("%d  x=%f y=%f time %f tired %f route = %d cost = %f\n",i, age[i].x[0], age[i].x[1], age[i].time, age[i].fatigue, age[i].t.route_num, age[i].route_cost[age[i].t.route_num]);
    count[age[i].t.route_num]++;
  }

  //各トリップごとのエージェント数を出力
  for(i=0;i<NUM_ROUTE_TYPE;i++){
    fprintf(fp2,"%d %d\n",i+1, count[i]);
  }
/*
  for(i=0;i<NUM_PARKING;i++){//駐車場ごとの駐車台数
    if(park[i].capacity != 0){
      fprintf(fp1,"%4d %3d %.0lf\n",i,park[i].num_park,(double)(park[i].num_park/park[i].capacity)*100);
      count_par++;
    }
  }
  fprintf(fp1,"\n%d/%d  %.0lf",(NUM_PARKING-count_par),NUM_PARKING,(double)(NUM_PARKING-count_par)/NUM_PARKING*100);//残った駐車場の数
*/





//居住地選択

	//モビリティマネジメント
  mobility_management(age);

  for(i=0;i<NUM_ROUTE_TYPE;i++){
    count[i]=0;
  }
  for(i=0;i<NUM_AGENT;i++){
    count[age[i].t.route_num]++;
  }
 
  fprintf(fp2,"\n");
      
  //各トリップごとのエージェント数
	for(i=0;i<NUM_ROUTE_TYPE;i++){
    fprintf(fp2,"%d %d\n",i+1, count[i]);
  }
   
  //転居回数
  printf("転居試行回数:");
  scanf("%d",&x);

  fprintf(fp9,"i homt_price,cost_home time money fatigue\n");
  
  //100エージェントずつ転居,候補先はそれぞれ10箇所
	for(h=0;h<x;h++){
		for(counter=0;counter<NUM_AGENT;counter+=100){
			printf("\n%dcycle%d～%dエージェント転居\n",h,counter,counter+100);

      //転居候補選択
			reset_home(hom,homt);
			//地代計算
			home_price(hom,homt,com);

			//交通移動
			for(k=0;k<1000;k+=100){

        //再度初期化必要な物初期化
				reset_agent(age, hom);

        //100エージェントが仮転居
				for(j=counter;j<counter+100;j++){
					age[j].x[0]=homt[(j-counter)+k].x[0];
					age[j].x[1]=homt[(j-counter)+k].x[1];
					//printf("%d %lf %lf\n",j,age[j].x[0],age[j].x[1]);
				}

        //仮転居先から目的地への移動
       MoveToDestination(age,unit,com,park,lot,next);

				//コストの算出
				for(j=counter;j<counter+100;j++){
					money(age, park, lot, s_lot, j);
					fprintf(fp9,"%d %.0lf %.0lf %.0lf %.0lf %.0lf\n",(j-counter)+k,homt[j].price,cost_home(age,j),age[j].time*	change_t,age[j].money,age[j].fatigue*change_f);
					
					//転居コスト=地代+移動コスト    
					homt[(j-counter)+k].price = homt[(j-counter)+k].price/360 + cost_home(age,j);
				}
	
			}
      
      //転居先選択+転居
			select_home(hom,homt,counter,fp8);
		   
			fprintf(fp7,"\n%dcycle%d～%dエージェント転居\n",h,counter,counter+100);
			fprintf(fp9,"\n%dcycle%d～%dエージェント転居\n",h,counter,counter+100);
			fprintf(fp8,"\n%dcycle%d～%dエージェント転居\n",h,counter,counter+100);
			for(i=0;i<NUM_HOME;i++){
				fprintf(fp7,"%d %.0lf %.0lf\n",i,hom[i].x[0],hom[i].x[1]);
			}
		
		}
	}
	
//全エージェント転居終了後の総コスト＋CO2算出

	//地代計算	
	home_price(hom,homt,com);
	
	//再度初期化必要な物初期化	reset_agent(age, hom);
	
	//転居先から目的地への移動
  MoveToDestination(age,unit,com,park,lot,next);

	//コストの算出
	for(i=0;i<NUM_AGENT;i++){
	  money(age, park, lot, s_lot, i);
	  fprintf(fp10,"%d, %.0lf, %.0lf, %.0lf, %.0lf, %.0lf, %.0lf\n",i,hom[i].price,cost_home(age,i),age[i].time*change_t,age[i].money,age[i].fatigue*change_f,hom[i].price/360+cost_home(age,i));
	  fprintf(fp11,"%d, %.2lf, %.2lf, %.2lf, %.2lf\n",i,age[i].distance_c/1000*100,age[i].distance_w/1000*0,age[i].distance_b/1000*0,age[i].distance_t/1000*30);//co2=移動距離×A walkとbikeは0
	  fprintf(fp12,"%d, %.2lf, %.2lf, %.2lf, %.2lf\n",i,age[i].distance_c/1000*100,age[i].distance_w/1000*2,age[i].distance_b/1000*8,age[i].distance_t/1000*6);//必要空間=移動距離×A
	  //homt[(j-counter)+k].price = homt[(j-counter)+k].price/360 + cost_home(age,j);// *50/365 移動費から削る/1000
	}

/*
//交通行動選択2回目
  
  fprintf(fp2,"\n--2--\n\n");
  
  //初期化
  E=0.7;  
  for(i=0;i<NUM_AGENT;i++){
    for(k=0;k<NUM_ROUTE_TYPE;k++){
      age[i].route_cost[k] = 0;
     }
   } 

  printf("交通行動試行回数:");
  scanf("%d",&x);
     
  //試行回数ループ
  for(h=0;h<x;h++){
  
    //再度初期化必要な物初期化
    reset_agent(age, hom);
    
    //駐車場の仮想容量設定
    for(i=0;i<NUM_PARKING;i++){
      park[i].capacity = 4;
    }
    
    printf("-------------%d日目--------------\n",h);
    
     //エージェントすべて移動終了までループ
    MoveToDestination(age,unit,com,park,lot,next);

    //移動総コストの算出
    for(k=0;k<NUM_AGENT;k++){
      money(age, park, lot, s_lot, k);    
      cost(age, k);
    }
    
    //駐車数が０な駐車場倒産.倒産時はcapacity=0
    for(i=0;i<NUM_PARKING;i++){
      if(park[i].num_park == 0)
        park[i].capacity = 0;
    }
    
    //エージェントのトリップ選択
    if(h != (x-1))
      select_rute(age);
  
     //10エージェントの1日ごとのトリップ選択変化
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

  //各トリップごとのエージェント数を出力
  for(i=0;i<NUM_ROUTE_TYPE;i++){
    fprintf(fp2,"%d %d\n",i+1, count[i]);
  }
	*/
  return 0;
}



//エージェント初期化
void setup_agent(struct agent a[], struct home h[]){
  int x, y, z;
  int p, q;               
  int i, j, k;               //ループ

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


    //乱数で偏向を設定
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
    //トリップの割り当て
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


//自宅初期化
void setup_home(struct home h[]){//(-3600～-400)

  int i,j,k,l,n;
  double m=-L/2.0;          // 平均m=Lh
  double sigma=Lh/5.0;  // 標準偏差σ=2*Lh/10 
  double r1,r2;
  //double limit,limit1,limit2;
  double x_sum=0,y_sum=0,sum2_x=0,sum2_y=0; 

  //int counter[101][101]={0};
  FILE *fp6;

	//srand((unsigned)time(NULL));       /* 乱数の初期化 */

  for(i=0;i<NUM_HOME;i+=2){

    //x座標
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
		//printf("%lf %lf\n",r1,r2);
    h[i].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2)+m);
    h[i+1].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2)+m);
	
    //y座標
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    h[i].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2));
    h[i+1].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2));

		x_sum+=h[i].x[0]+h[i+1].x[0];//平均
		y_sum+=h[i].x[1]+h[i+1].x[1];
		sum2_x=(h[i].x[0]+L/2)*(h[i].x[0]+L/2)+(h[i+1].x[0]+L/2)*(h[i+1].x[0]+L/2);//標準偏差
		sum2_y=h[i].x[1]*h[i].x[1]+h[i+1].x[1]*h[i+1].x[1];
	}
/*
	for(i=0;i<NUM_HOME;i+=2){
		h[i].x[0] = h[i].x[0] - Lh - L/2;
    h[i+1].x[0] = h[i+1].x[0] - Lh - L/2;
    x_sum+=h[i].x[0]+h[i+1].x[0];//平均
  }
*/
  for(i=0;i<NUM_HOME;i++){
		if(h[i].x[0]<-10000 || h[i].x[1]<-10000 || 10000 < h[i].x[0] || 10000 < h[i].x[1]){
      printf("error!!%d %f %f \n",i,h[i].x[0], h[i].x[1]);
		}
  }
  printf("x_average=%lf\ny_average=%lf\n",x_sum/NUM_HOME,y_sum/NUM_HOME);//平均
  printf("x_deviation=%lf\ny_debiation=%lf\n",sqrt(sum2_x/NUM_HOME),sqrt(sum2_y/NUM_HOME));//標準偏差



  //分布図
  if((fp6 = fopen("home0.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }


	for(i=0;i<NUM_HOME;i++){
		fprintf(fp6,"%d %.0lf %.0lf\n",i,h[i].x[0],h[i].x[1]);

	}
  
}
//仮転居先(自宅)初期化
void reset_home(struct home h[],struct home_temp ht[]){
  
	int i,j,k,l,n;
  double sum_x=0,sum_y=0;
	double sum2_x=0,sum2_y=0;
	double average_x=0,average_y=0;
	double deviation_x=0,deviation_y=0;
	double r1,r2;


  //現在の平均・分散算出
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

  printf("x_average=%lf\ny_average=%lf\n",average_x,average_y);//平均
  printf("x_deviation=%lf\ny_debiation=%lf\n\n",deviation_x,deviation_y);//標準偏差

  sum_x=0;
	sum_y=0;
  sum2_x=0;
	sum2_y=0;

  //仮転居先(10箇所*100エージェント)決定
  for(i=0;i<1000;i+=2){

    //x座標
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
		
    ht[i].x[0] = (int)(deviation_x*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2)+average_x);
    ht[i+1].x[0] = (int)(deviation_x*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2)+average_x);
	
    //y座標
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    ht[i].x[1] = (int)(deviation_y*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2)+average_y);
    ht[i+1].x[1] = (int)(deviation_y*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2)+average_y);

		sum_x+=ht[i].x[0]+ht[i+1].x[0];//平均
		sum_y+=ht[i].x[1]+ht[i+1].x[1];
		sum2_x=(ht[i].x[0]-average_x)*(ht[i].x[0]-average_x)+(ht[i+1].x[0]-average_x)*(ht[i+1].x[0]-average_x);//標準偏差
		sum2_y=(ht[i].x[1]-average_y)*(ht[i].x[1]-average_y)+(ht[i+1].x[1]-average_y)*(ht[i+1].x[1]-average_y);
	}

/*
	for(i=0;i<1000;i+=2){
		ht[i].x[0] = ht[i].x[0] - Lh - L/2;
    ht[i+1].x[0] = ht[i+1].x[0] - Lh - L/2;
    sum_x+=ht[i].x[0]+ht[i+1].x[0];//平均
  }
*/
	
/*
	for(i=0;i<1000;i++){
		printf("%d %.lf %.lf\n",i,ht[i].x[0],ht[i].x[1]);
	}
	*/
/*
  printf("x_average=%lf\ny_average=%lf\n",sum_x/1000,sum_y/1000);//平均
  printf("x_deviation=%lf\ny_debiation=%lf\n",sqrt(sum2_x/1000),sqrt(sum2_y/1000));//標準偏差
*/
}


//地代計算
void home_price(struct home h[],struct home_temp ht[],struct company c[]){

  int x,y,i,j,k;
	double A=30000*140*100;//地代係数η*１世帯面積50(*100先行研究と実際数の差×）(代わりに移動費＋*1000or先行研究と今の設定面積の差)
	double H=100;       //会社地代の重み=2

 
  for(j=0;j<(Yh-Yl)/100;j++){
	  for(k=0;k<(Xh-Xl)/100;k++){
      home_dencity[j][k]=0;
			com_dencity[j][k]=0;
	  }
  }

	//homeの密度計算
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

	//comの密度計算
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


	//転居候補の地代算出
	for(i=0;i<1000;i++){                         //i= 100エージェント*10箇所
    for(j=0,y=Yl ; y<=Yh ; j++,y+=100){
			if(y<=ht[i].x[1] && ht[i].x[1]<y+100){
	      for(k=0,x=Xl ; x<=Xh ; k++,x+=100){			
			    if(x<=ht[i].x[0] && ht[i].x[0]<x+100){
					  ht[i].price = A*(home_dencity[j][k]+H*com_dencity[j][k])/10000;//地代(仮転居先)
					  //fprintf(fp8,"%d %lf \n",i,ht[i].price);			
	        }
			  }
		  }
	  }
	}
	
	//自宅位置の地代算出
	for(i=0;i<NUM_HOME;i++){                         
    for(j=0,y=Yl ; y<=Yh ; j++,y+=100){
			if(y<=h[i].x[1] && h[i].x[1]<y+100){
	      for(k=0,x=Xl ; x<=Xh ; k++,x+=100){			
			    if(x<=h[i].x[0] && h[i].x[0]<x+100){
					  h[i].price = A*(home_dencity[j][k]+H*com_dencity[j][k])/10000;//地代(自宅)
					  //fprintf(fp8,"%d %lf \n",i,ht[i].price);			
	        }
			  }
		  }
	  }
	}

}

//転居先選択
void select_home(struct home h[],struct home_temp ht[],int counter,FILE *fp8){

	int i,j;
  double best=99999999; 
  int best_num;

	for(i=counter;i<counter+100;i++){//h[]:エージェント番号,ht[]:一の位・十の位
		best=99999999;
		for(j=0;j<1000;j+=100){//百の位
		  if(ht[(i-counter)+j].price<best){
	      h[i].x[0]=ht[(i-counter)+j].x[0];
        h[i].x[1]=ht[(i-counter)+j].x[1];
				 best=ht[(i-counter)+j].price;
				 best_num=(i-counter)+j;
		  }
	  }
		fprintf(fp8,"%d %.0lf %d\n",i,best,best_num);//エージェント番号　転居コスト
	}
  //fprintf(fp8,"\ncycle\n");
}

//モビリティマネジメント
void mobility_management(struct agent a[]){

  int i,j;
  int counter=0;
  int q;

	for(i=0;i<NUM_AGENT;i++){
		if(a[i].t.route_num==3 || a[i].t.route_num==5){
			counter++;
		}
	}
	
	//○%を自動車利用から電車利用へ
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

//会社(目的地)初期化
void setup_company(struct company c[]){
  int i,j,k,l,n;
  double m=L/2;          // 平均m=Lb
  double sigma=Lb/5.0;  // 標準偏差σ=2*Lb/10 
  double r1,r2;
  double limit;
  double x_sum=0,y_sum=0,sum2_x=0,sum2_y=0; 

  
  FILE *fp5;
  

  for(i=0;i<NUM_COMPANY;i+=2){

    //x座標
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    c[i].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2)+m);
    c[i+1].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2)+m);
	
    //y座標
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    c[i].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2));
    c[i+1].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2));

		x_sum+=c[i].x[0]+c[i+1].x[0];//平均
    y_sum+=c[i].x[1]+c[i+1].x[1];
	  sum2_x=(c[i].x[0]+2000)*(c[i].x[0]+2000)+(c[i+1].x[0]+2000)*(c[i+1].x[0]+2000);//標準偏差
	  sum2_y=c[i].x[1]*c[i].x[1]+c[i+1].x[1]*c[i+1].x[1];
  }
/*
  for(i=0;i<NUM_COMPANY;i+=2){
    c[i].x[0] = c[i].x[0] - Lb + L/2;
    c[i+1].x[0] = c[i+1].x[0] - Lb + L/2;
    x_sum+=c[i].x[0]+c[i+1].x[0];//平均
  }
	*/

/*
  for(i=0;i<NUM_COMPANY;i++){
    printf("%d %f %f \n",i,c[i].x[0], c[i].x[1]);
  }
  printf("x_average=%lf\ny_average=%lf\n",x_sum/NUM_COMPANY,y_sum/NUM_COMPANY);//平均
  printf("x_deviation=%lf\ny_debiation=%lf\n",sqrt(sum2_x/NUM_COMPANY),sqrt(sum2_y/NUM_COMPANY));//標準偏差
*/

  //分布図
  if((fp5 = fopen("business.csv", "w")) == NULL) {
    printf("file open error!!\n");
    exit(EXIT_FAILURE);
  }
  
	for(i=0;i<NUM_COMPANY;i++){
	  fprintf(fp5,"%d %.0lf %.0lf\n",i,c[i].x[0],c[i].x[1]);
	}

}  

//駐輪場,駐車場初期化
void setup_park_lot(struct parking p[], struct bikelot l[], struct stationlot sl){

  int i,j,k,o,n;
  double m=L/2;          // 平均m=Lb
  double sigma=Lb/5.0;  // 標準偏差σ=2*Lb/10 
  double r1,r2;
  double limit;
  double x_sum=0,y_sum=0,sum2_x=0,sum2_y=0; 



  for(i=0;i<NUM_PARKING;i+=2){

    //x座標
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    p[i].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2)+m);
    p[i+1].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2)+m);
	
    //y座標
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    p[i].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2));
    p[i+1].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2));
    
		x_sum+=p[i].x[0]+p[i+1].x[0];//平均
    y_sum+=p[i].x[1]+p[i+1].x[1];
	  sum2_x=(p[i].x[0]+2000)*(p[i].x[0]+2000)+(p[i+1].x[0]+2000)*(p[i+1].x[0]+2000);//標準偏差
	  sum2_y=p[i].x[1]*p[i].x[1]+p[i+1].x[1]*p[i+1].x[1];

	  x_sum+=l[i].x[0]+l[i+1].x[0];//平均
    y_sum+=l[i].x[1]+l[i+1].x[1];//平均
	  sum2_x=(l[i].x[0]+2000)*(l[i].x[0]+2000)+(l[i+1].x[0]+2000)*(l[i+1].x[0]+2000);//標準偏差
	  sum2_y=l[i].x[1]*l[i].x[1]+l[i+1].x[1]*l[i+1].x[1];
  }


  for(i=0;i<NUM_BIKELOT;i+=2){

    //x座標
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    l[i].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2)+m);
    l[i+1].x[0] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2)+m);
	
    //y座標
    r1 = (double)rand()/RAND_MAX;
    r2 = (double)rand()/RAND_MAX;
    l[i].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*cos(2.0*M_PI*r2));
    l[i+1].x[1] = (int)(sigma*sqrt(-2.0*log(r1))*sin(2.0*M_PI*r2));

	  x_sum+=l[i].x[0]+l[i+1].x[0];//平均
    y_sum+=l[i].x[1]+l[i+1].x[1];//平均
	  sum2_x=(l[i].x[0]+2000)*(l[i].x[0]+2000)+(l[i+1].x[0]+2000)*(l[i+1].x[0]+2000);//標準偏差
	  sum2_y=l[i].x[1]*l[i].x[1]+l[i+1].x[1]*l[i+1].x[1];
  }

  for(i=0;i<NUM_BIKELOT;i+=2){
    l[i].x[0] = l[i].x[0] - Lb + L/2;
    l[i+1].x[0] = l[i+1].x[0] - Lb + L/2;
    x_sum+=l[i].x[0]+l[i+1].x[0];//平均
  }


  for(i=0;i<NUM_PARKING;i++){
 
    p[i].capacity = 4;               //容量設定    
    p[i].charge = 800;
    p[i].num_park = 0;
  }

  for(i=0;i<NUM_BIKELOT;i++){    

    l[i].capacity = 4;             //容量設定
    l[i].charge = 150;
    l[i].num_lot = 0;
  }
  
  sl.capacity = 4; 
  sl.charge = 150; 
  sl.num_lot =0;
  
}

//トリップの道筋
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

  //違法駐輪
  route[6][0] = B_COMPANY;    route[6][1] = END;
  route[7][0] = B_STATION_A;  route[7][1] = T_STATION_B;  route[7][2] = W_COMPANY;  route[7][3]= END;
}

//エージェント再初期化
void reset_agent(struct agent a[],struct home h[]){
  int i;               //ループ
  
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

//エージェントがすべて移動終了するまでループ
void MoveToDestination(struct agent a[],struct unit_trip u[],struct company c[],struct parking p[],struct bikelot bl[],double next[]){

  int i;

  while(check < NUM_AGENT){
    //車密度の計算
	  calc_dencity(a, u);
			
    for(i=0;i<NUM_AGENT;i++){    
	     //移動終了か否か
	    if(a[i].t.unit[a[i].t.unit_state] != 100){
	       //エージェントの速度の更新  
	      a[i].v = speed(u[a[i].t.unit[a[i].t.unit_state]].method);
	        
				 //渋滞速度・疲れの加算			  
				 congestion(a, u, i);
					
	       //次の目標地点を設定
	      next_spot(a, c, p, bl, u, next, i);
	       //移動
	      movement(a, next, u, i);
	        		
	       //疲れの計算
	      tired(a ,u, i);
	       //目標地点へ到着か否か
	      if(check_arrival(a, next, i)){
	         //前の目的地が駐車場なら   
          if(a[i].t.unit[a[i].t.unit_state-1] == C_PARKING){
              //駐車場仮想容量を減らす           
            p[a[i].parking].num_park++;   
           }
         }
 			 }  
		 }
  }
}

//トリップの割り当て
void select_rute(struct agent a[]){
  
  int i, h, q, j;
  int best;
  
	//srand((unsigned)time(NULL));       /* 乱数の初期化 */

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

//交通機関の速度
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

  return 0;//どれも当てはまらなかったとき
}

//次の目標地点の割り当て
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

  if(u[a[i].t.unit[a[i].t.unit_state]].goal_id == HIGHWAY_E){//幹線道路上の駐車場x座標まで移動.利用する駐車場はここで決めておく
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

//移動
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
  if(u[a[i].t.unit[a[i].t.unit_state]].method == CAR_H){//ここを使って分担率の計算?.
    count_CAR_H++;
  }
  if(u[a[i].t.unit[a[i].t.unit_state]].method == CAR){
    count_CAR++;
  }
	*/
} 

//到着確認//到着時に1を返すように変更
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

//交通機関による疲れ計算
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

//トリップにかかった金額
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

//コスト算出
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

//仮転居後の移動コスト算出
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

//車密度計算
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
						    a[i].d_num[0]=j;//carとcar_hのみ初期化．
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

//渋滞による速度変化・疲れ加算
void congestion(struct agent a[], struct unit_trip u[], int i){//追加必要 
	  
	  double temp;


//printf("%d %d %d\n",i,a[i].d_num[0],a[i].d_num[1]);
		temp=a[i].fatigue;
	  if(u[a[i].t.unit[a[i].t.unit_state]].method == CAR_H || u[a[i].t.unit[a[i].t.unit_state]].method == CAR){
			//printf("%.1lf\n",(-1.14*(car_dencity[a[i].d_num[0]][a[i].d_num[1]]-25)+80)*1000/60);
		  if((-1.14*(car_dencity[a[i].d_num[0]][a[i].d_num[1]]-25)+80)*1000/60 < a[i].v){//セグメントエラーa[i].d_numがおかしい．
				//printf("%.1lfm/min\n",(-1.14*(car_dencity[a[i].d_num[0]][a[i].d_num[1]]-25)+80)*1000/60);
				a[i].v=(-1.14*(car_dencity[a[i].d_num[0]][a[i].d_num[1]]-25)+80)*1000/60;//速度
				a[i].fatigue += car_dencity[a[i].d_num[0]][a[i].d_num[1]]*0.1;//疲れ
      
				if(a[i].v <= 0){
					a[i].v=83;//最低速度5km/h 83m/分
					a[i].fatigue += 10-car_dencity[a[i].d_num[0]][a[i].d_num[1]]*0.1;//最大疲れ10
				}
				//printf("%d %d %d  %.1lfkm/h 疲れ%.1lf\n\n",i,a[i].d_num[0],a[i].d_num[1],a[i].v*60/1000,a[i].fatigue-temp);
			}
		}

}

//駐輪場の選択
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

//駐車場の選択
int selectparking(struct company c[],struct agent a[], struct parking p[], int i){
  int h, l,lx, ly, best_num;
  double  best=99999999;
  double temp1,temp3;

  for(h=0;h<NUM_PARKING;h++){
    if(p[h].capacity != 0){//駐車場が倒産してなければ
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
