#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <algorithm>
using namespace std;
#define endl '\n'
#define NUM_OF_NODE 5000					//The number of nodes
#define THREEOPTTIME 10.0
#define TWOOPTTIME 25.0
#define THREEOPTTIME2 29.9

string input_txt = "pbk411.txt";			//input txt file //dcb2086.txt //lsn3119.txt //pbk411.txt  //xqf131.txt //fjs3649.txt //dkc3938.txt //xmc10150.txt
double dropout = 0.25;

struct Edge {
	double cost;
	int start, end;
};

bool cmp(const Edge& e1, const Edge& e2) {
	return e1.cost < e2.cost;
}

Edge e[NUM_OF_NODE*NUM_OF_NODE / 2];
int parent[NUM_OF_NODE];
int node[NUM_OF_NODE][2];							//tsp nodes
double dist[NUM_OF_NODE][NUM_OF_NODE];				//distances from node to node

int opt_number = 1;
clock_t c_start, c_end;
double exe_time;

int Find(int x) {
	if (x == parent[x])
		return x;
	else
		return parent[x] = Find(parent[x]);
}

void Union(int x, int y) {
	x = Find(x);
	y = Find(y);
	parent[x] = y;
}

void readFile(int *len) {
	ifstream fin(input_txt);
	string s;
	getline(fin, s);
	//find length of Node
	for (int i = 0; i < 7; i++)
		getline(fin, s, ' ');
	fin >> *len;
	if (*len < 0)
		cout << "length_error" << endl;
	//dummy data
	while (s != "NODE_COORD_SECTION") {
		getline(fin, s);
	}
	//input Node 1 ~ len 
	for (int i = 0; i < *len; i++) {
		int a, x, y;
		fin >> a >> node[i + 1][0] >> node[i + 1][1];
	}
	fin.clear();
	fin.close();	//close input data stream
}

void printPath(int len, int *route) {
	ofstream fout("result.txt");
	for (int i = 0; i <= len; i++) {
		fout << route[i] << endl;
	}
	cout << "result.txt file is generated" << endl;
}

int calcDist(int len) {
	int e_count = 0;
	for (int i = 1; i <= len; i++) {
		for (int j = i + 1; j <= len; j++) {
			int x = node[i][0] - node[j][0];
			int y = node[i][1] - node[j][1];
			int r = x * x + y * y;
			double w = sqrt(r);
			dist[i][j] = w;
			dist[j][i] = w;
		}
	}
	double max_x = -1;
	for (int j = 2; j <= len; j++) {
		if (max_x < dist[1][j])
			max_x = dist[1][j];
	}
	for (int i = 1; i <= len; i++) {
		for (int j = i + 1; j <= len; j++) {
			if (dist[i][j] < max_x * dropout)
				e[e_count++] = { dist[i][j],i,j };
		}
	}
	return e_count;
}

double calDist(int p1_x, int p1_y, int p2_x, int p2_y) {
	int x = p1_x - p2_x;
	int y = p1_y - p2_y;
	double dist = sqrt(x*x + y * y);

	return dist;
}

double sumPath(int len, int *rt) {
	double sum = 0;
	for (int i = 0; i < len; i++) {
		sum += dist[rt[i]][rt[i + 1]];
	}
	return sum;
}

void EdgeFirst(int e_len, int len, int * route) {
	vector<int> v_node[NUM_OF_NODE];
	bool chk[NUM_OF_NODE];
	memset(chk, 0, sizeof(chk));
	for (int i = 1; i <= len; i++)
		parent[i] = i;
	for (int i = 0; i < e_len; i++) {
		int u = e[i].start;
		int v = e[i].end;
		int x = Find(u);
		int y = Find(v);
		if (x != y) {
			if (v_node[u].size() < 2 && v_node[v].size() < 2) {
				Union(u, v);
				v_node[u].push_back(v);
				v_node[v].push_back(u);
			}
		}
	}
	int no = 0;
	vector<int> no_v;
	for (int i = 1; i <= len; i++) {
		if (v_node[i].size() < 2) {
			no++;
			no_v.push_back(i);
		}
	}

	if (no == 2) {
		int last[3];
		int last_i = 0;
		for (int i = 1; i <= len; i++) {
			if (v_node[i].size() < 2)
				last[last_i++] = i;
		}
		v_node[last[0]].push_back(last[1]);
		v_node[last[1]].push_back(last[0]);
	}
	else {
		cout << "additional edge" << endl;
		int n_len = no_v.size();
		for (int i = 0; i < n_len; i++) {
			for (int j = i + 1; j < n_len; j++) {
				int u = no_v[i];
				int v = no_v[j];
				int x = Find(u);
				int y = Find(v);
				if (x != y) {
					if (v_node[u].size() < 2 && v_node[v].size() < 2) {
						Union(u, v);
						v_node[u].push_back(v);
						v_node[v].push_back(u);
						break;
					}
				}
			}
		}
		int last[3];
		int last_i = 0;
		for (int i = 1; i <= len; i++) {
			if (v_node[i].size() < 2)
				last[last_i++] = i;
		}
		v_node[last[0]].push_back(last[1]);
		v_node[last[1]].push_back(last[0]);
	}
	int start_p = 1;
	route[0] = start_p;
	chk[start_p] = true;
	int p = start_p;
	int count = 1;
	for (int i = 1; i <= len; i++) {
		int n_size = v_node[p].size();
		int j = 0;
		while (n_size--) {
			if (!chk[v_node[p][j]]) {
				chk[v_node[p][j]] = true;
				route[count++] = v_node[p][j];
				p = v_node[p][j];
			}
			else
				j++;
		}
	}
	route[count++] = start_p;
}

void indexAcs(int len, int* route) {
	route[0] = 1;
	for (int i = 1; i < len; i++) {
		route[i] = i+1;
	}
	route[len] = 1;
}

void ZigZagToggle_vertical_first(int len, int* route) {
	int node_temp[NUM_OF_NODE];
	int k = 0;
	int top = 0;
	double front, behind;
	bool toggle = true; // true만 아래서 위로 , false면 위에서 아래로
	bool first_col = false;

	for (int i = 1; i <= len; i++) {
		if (node[i][0] == node[i + 1][0]) { // 예외처리 필요 len+1이 없다.
			node_temp[k] = i;
			k++;
		}
		else {
			node_temp[k] = i;
			if (top > 0) {
				front = calDist(node[node_temp[0]][0], node[node_temp[0]][1], node[route[top - 1]][0], node[route[top - 1]][1]);
				behind = calDist(node[node_temp[k]][0], node[node_temp[k]][1], node[route[top - 1]][0], node[route[top - 1]][1]);
			}
			else {
				front = calDist(node[node_temp[0]][0], node[node_temp[0]][1], node[i + 1][0], node[i + 1][1]);
				behind = calDist(node[node_temp[k]][0], node[node_temp[k]][1], node[i + 1][0], node[i + 1][1]);
				if (front < behind)
					first_col = true;
			}
			if (front<behind || first_col == false) {
				for (int j = 0; j <= k; j++) {
					route[top] = node_temp[j];
					top++;
				}
				toggle = false;
			}
			else {
				for (int j = k; j >= 0; j--) {
					route[top] = node_temp[j];
					top++;
				}
				toggle = true;
			}
			k = 0;
		}
	}
	route[len] = len;
}

void ZigZagToggle_vertical_last(int len, int* route) {
	int node_temp[NUM_OF_NODE];
	int k = 0;
	int top = 0;
	bool toggle = true; // true만 아래서 위로 , false면 위에서 아래로

	for (int i = len; i >= 1; i--) {
		if (node[i - 1][0] == node[i][0]) { // 예외처리 필요 len+1이 없다.
			node_temp[k] = i;
			k++;
		}
		else {
			node_temp[k] = i;
			if (toggle == true) {
				for (int j = 0; j <= k; j++) {
					route[top] = node_temp[j];
					top++;
				}
				toggle = false;
			}
			else {
				for (int j = k; j >= 0; j--) {
					route[top] = node_temp[j];
					top++;
				}
				toggle = true;
			}
			k = 0;
		}
		if (i == 1) {
			if (toggle == true) {
				for (int j = 0; j <= k; j++) {
					route[top] = node_temp[j];
					top++;
				}
				toggle = false;
			}
			else {
				for (int j = k; j >= 0; j--) {
					route[top] = node_temp[j];
					top++;
				}
				toggle = true;
			}
		}
	}
	route[len] = len;
}

void NearestNeighbor(int len, int * route, int start) {
	bool chk[NUM_OF_NODE];						//chk is visited
	memset(chk, 0, sizeof(chk));
	int startPoint = start;
	route[0] = startPoint;
	chk[startPoint] = true;
	int cnt = 1;
	while (1) {
		int min = -1;
		int next = 0;
		for (int i = 1; i <= len; i++) {
			if (chk[i])
				continue;
			if (min == -1 || min > dist[route[cnt - 1]][i]) {
				min = dist[route[cnt - 1]][i];
				next = i;
			}
		}
		route[cnt++] = next;
		chk[next] = true;
		if (cnt == len)
			break;
	}
	route[cnt] = startPoint;
}

void randRoute(int len, int* route) {
	bool chk[NUM_OF_NODE];

	for (int k = 0; k < 5; k++) {
		int r_tmp = rand() % len + 1;
		memset(chk, 0, sizeof(chk));
		int start = r_tmp;
		route[k] = start;
		chk[start] = true;
		int count = 1;
		while (1) {
			int min = -1;
			int next = 0;
			for (int i = 1; i <= len; i++) {
				if (chk[i])
					continue;
				if (min == -1 || min > dist[route[count - 1]][i]) {
					min = dist[route[count - 1]][i];
					next = i;
				}
			}
			route[k] = next;
			chk[next] = true;
			count++;
			if (count == len)
				break;
		}
		route[len] = start;
	}
}

void random(int len, int *rt) {
	bool chk[NUM_OF_NODE];
	memset(chk, 0, sizeof(chk));
	int cnt = 0;
	while (cnt != len) {
		int tmp = rand() % len + 1;
		if (!chk[tmp]) {
			rt[cnt++] = tmp;
			chk[tmp] = true;
		}
	}
	rt[cnt] = rt[0];
}

void mix_route(int len, int start, int end, int* rt1,int* rt2) {
	int temp;
	int* pair1;
	int* pair2;
	double rand_num;

	pair1 = (int*)malloc(sizeof(int*)*(len+1));
	pair2 = (int*)malloc(sizeof(int*)*(len + 1));

	/* start부터 end까지 바꾸고 pair 맺어줌 */
	for (int i = start; i <= end ; i++) {
		temp = rt1[i];
		rt1[i] = rt2[i];
		rt2[i] = temp;
		pair1[rt1[i]] = rt2[i]; // 인덱스 : rt1, 값 : rt2
		pair2[rt2[i]] = rt1[i];
	}

	for (int i = 0; i < start; i++) {
		if (pair1[rt1[i]] <= len && pair1[rt1[i]] > 0) {
			rt1[i] = pair1[rt1[i]];
		}
		if (pair2[rt2[i]] <= len && pair2[rt2[i]] > 0) {
			rt2[i] = pair2[rt2[i]];
		}
	}

	for (int i = end + 1; i < len; i++) {
		if (pair1[rt1[i]] <= len && pair1[rt1[i]] > 0) {
			rt1[i] = pair1[rt1[i]];
		}
		if (pair2[rt2[i]] <= len && pair2[rt2[i]] > 0) {
			rt2[i] = pair2[rt2[i]];
		}
	}

	rt1[len] = rt1[0];
	rt2[len] = rt2[0];

	rand_num = rand() / RAND_MAX; // 1번 rt에서 돌연변이
	if (rand_num < 0.01) {
		int rand_index = rand() % (len - 3) + 2;
		temp = rt1[rand_index];
		rt1[rand_index] = rt1[rand_index+1];
		rt1[rand_index + 1] = temp;
	}

	rand_num = rand() / RAND_MAX; // 2번 rt에서 돌연변이
	if (rand_num < 0.01) {
		int rand_index = rand() % (len - 3) + 2;
		temp = rt2[rand_index];
		rt2[rand_index] = rt2[rand_index + 1];
		rt2[rand_index + 1] = temp;
	}
}

int t[NUM_OF_NODE];

int main() {
	int route1[NUM_OF_NODE];						//tsp path array
	int route2[NUM_OF_NODE];
	int route3[NUM_OF_NODE]; 
	int route4[NUM_OF_NODE];
	int route5[NUM_OF_NODE];
	int route6[NUM_OF_NODE];
	int route7[NUM_OF_NODE];
	int route8[NUM_OF_NODE];
	int route9[NUM_OF_NODE];
	int route10[NUM_OF_NODE];
	int** route;

	double route_sum[10];
	double temp;
	int* temp_rt;

	cout << "genetic algorithm" << endl;
	srand(time(NULL));
	int len = -1;
	readFile(&len);
	cout << len << " node TSP" << endl;

	c_start = clock();

	route = (int**)malloc(sizeof(int*) * 10);

	route[0] = route1;
	route[1] = route2;
	route[2] = route3;
	route[3] = route4;
	route[4] = route5;
	route[5] = route6;
	route[6] = route7;
	route[7] = route8;
	route[8] = route9;
	route[9] = route10;

	//calculate distance from i-node to j-node 
	int e_count = calcDist(len);
	stable_sort(e, e + e_count, cmp);

	c_end = clock();
	exe_time = (double)(c_end - c_start) / CLOCKS_PER_SEC;
	cout << "Sorting time: " << exe_time << "seconds" << endl;

	//calculate first TSP Path
	EdgeFirst(e_count, len, route1);
	temp = rand() % len + 1;
	NearestNeighbor(len, route2, 1);
	temp = rand() % len + 1;
	NearestNeighbor(len, route3, 1);
	temp = rand() % len + 1;
	NearestNeighbor(len, route4, 1);
	temp = rand() % len + 1;
	NearestNeighbor(len, route5,1);
	temp = rand() % len + 1;
	NearestNeighbor(len, route6, temp);
	temp = rand() % len + 1;
	NearestNeighbor(len, route7, temp);
	temp = rand() % len + 1;
	NearestNeighbor(len, route8, temp);
	temp = rand() % len + 1;
	NearestNeighbor(len, route9, temp);
	temp = rand() % len + 1;
	NearestNeighbor(len, route10, temp);

/*	indexAcs(len, route2);
	ZigZagToggle_vertical_first(len, route3);
	ZigZagToggle_vertical_last(len, route4);
	random(len, route6);
	random(len, route7);
	random(len, route8);
	random(len, route9);
	random(len, route10); */

	for (int i = 0; i < 10; i++) {
		route_sum[i] = sumPath(len, route[i]);
	}

	for (int i = 0; i < 10; i++) {
		cout << "route" <<i<<" finish " << route_sum[i] << endl;
	}

	//GA start
	cout << "GA Start!" << endl;
	int k = 1;
	while (1) {
		if ((double)(clock() - c_start) / CLOCKS_PER_SEC >= 120) {
			break;
		}
		
		int start = rand() % (len-1) + 1;
		int end = rand() % (len-1) + 1;

		if (start > end) {
			int temp = end;
			end = start;
			start = temp;
		}

		/* selection sort */
		for (int i = 0; i < 10; i++) {
			for (int j = i; j < 10; j++) {
				if (route_sum[i] > route_sum[j]) {
					temp = route_sum[i];
					route_sum[i] = route_sum[j];
					route_sum[j] = temp;

					temp_rt = route[i];
					route[i] = route[j];
					route[j] = temp_rt;
				}
			}
		}

		for (int i = 0; i < 5; i++) {
			mix_route(len, start, end, route[i*2], route[i*2 + 1]);
		}

		for (int i = 0; i < 10; i++) {
			route_sum[i] = sumPath(len, route[i]);
		}
		
		for (int i = 0; i < 10; i++) {
			cout << k << "번째 route" <<i+1 << " : " << route_sum[i] << endl;
		}
		k++;
	}

	//sum path
	for (int i = 0; i < 10; i++) {
		cout << "최종 route"<<i+1<<" : " << route_sum[i] << endl;
	}

	//print excute time
	c_end = clock();
	exe_time = (double)(c_end - c_start) / CLOCKS_PER_SEC;
	cout << "Execute time: " << exe_time << "seconds" << endl;

	//print PATH
	printPath(len, route[0]);
}
