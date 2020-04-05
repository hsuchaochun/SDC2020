# include <iostream>
# include <stdio.h>
# include <stdlib.h>
# include <time.h>
using namespace std;

enum weather{
    sunny = 0, cloudy, rainy
};

weather nextWeather(weather);
void printWeather(weather);

int main(void){
    srand(time(NULL));

// 2_b
    int todayWeather;
    int length;

    do{
        cout << "Please enter today's weather (sunny: 0, cloudy: 1, rainy: 2) -> ";
        cin >> todayWeather;
    }while(todayWeather != (int)todayWeather || todayWeather > 2 || todayWeather < 0);

    do{
        cout << "Enter the length of the weather sequence (0-100) -> ";
        cin >> length;
    }while(length != (int)length || length > 100 || length < 0);
    
    weather w = (weather)todayWeather;
    for(int i = 0; i < length; i++){
        printWeather((weather)w);
        w = nextWeather(w);
    }
    cout << endl;
// end of 2_b

// 2_c
    // int seq_duration = 10000;
    // int seq_length = 10000;
    // int sunny_count = 0;
    // int cloudy_count = 0;
    // int rainy_count = 0;

    // for(int i = 0; i < seq_duration; i++){
    //     weather w = (weather)(rand() % 3);
    //     for(int j = 0; j < seq_length; j++){
    //         if((weather)w == sunny) sunny_count++;
    //         else if((weather)w == cloudy) cloudy_count++;
    //         else rainy_count++;

    //         w = nextWeather(w);
    //     }
    // }

    // cout << "Prob. of sunny days:\t" << ((double)sunny_count / (seq_duration * seq_length)) << endl;
    // cout << "Prob. of cloudy days:\t" << ((double)cloudy_count / (seq_duration * seq_length)) << endl;
    // cout << "Prob. of rainy days:\t" << ((double)rainy_count / (seq_duration * seq_length)) << endl;
// end of 2_c

    return 0;
}

weather nextWeather(weather todayWeather){
    double trans[3][3] = {
        {0.8, 0.2, 0},
        {0.4, 0.4, 0.2},
        {0.2, 0.6, 0.2}
    };

    double num = (double)(rand() % 100) / 100;
    if(num < trans[todayWeather][0]) return sunny;
    else if(num < trans[todayWeather][0] + trans[todayWeather][1]) return cloudy;
    else return rainy;
}

void printWeather(weather w){
    if(w == sunny) cout << "sunny\t";
    else if(w == cloudy) cout << "cloudy\t";
    else cout << "rainy\t";
}