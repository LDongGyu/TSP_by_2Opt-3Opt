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

string input_txt = "lsn3119.txt";			//input txt file //dcb2086.txt //lsn3119.txt //pbk411.txt  //xqf131.txt //fjs3649.txt //dkc3938.txt //xmc10150.txt
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

void readFile(int *len);							//read data file
void printPath(int len, int *route);				//print approximately optimal path
int calcDist(int len);								//calculate distances from node to node
double sumPath(int len, int *route);				//sum tsp path
void twoOptSwap(int len, int a, int b, int *route);	//swap 2 edge
void twoOpt2(int len, int loof, int * rt);
void twoOpt(int len, int * rt);						//2-opt

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
			int r = x*x + y*y;
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

double sumPath(int len, int *rt) {
	double sum = 0;
	for (int i = 0; i < len; i++) {
		sum += dist[rt[i]][rt[i + 1]];
	}
	return sum;
}

void twoOptSwap(int len, int a, int b, int *route) {
	int new_route[NUM_OF_NODE];
	int n_i = 0;
	for (int i = 0; i < a; i++)
		new_route[n_i++] = route[i];
	for (int i = b; i >= a; i--)
		new_route[n_i++] = route[i];
	for (int i = b + 1; i <= len; i++)
		new_route[n_i++] = route[i];

	for (int i = 0; i <= len; i++)
		route[i] = new_route[i];
}

void twoOpt2(int len, int loof, int * rt) {
	int try_i = 0;
	while (try_i < loof) {
		c_end = clock();
		exe_time = (double)(c_end - c_start) / CLOCKS_PER_SEC;
		if (exe_time > TWOOPTTIME)
			break;

		int x = (rand() % (len - 2)) + 2;
		int y = (rand() % (len - 2)) + 2;
		if (x > y)
			swap(x, y);
		if (x == y)
			continue;

		double dist_o = dist[rt[x - 1]][rt[x]] + dist[rt[y]][rt[y + 1]];
		double dist_n = dist[rt[x - 1]][rt[y]] + dist[rt[x]][rt[y + 1]];
		double diff = dist_o - dist_n;

		if (diff > 0) {
			twoOptSwap(len, x, y, rt);
		//	cout << opt_number++ << "번째 개선" << endl;
		//	double sum = sumPath(len, rt);
		//	cout << "Sum: " << sum << endl;
		}
		try_i++;
	}
}

void twoOpt(int len, int * rt) {
	int try_i = 0;
	while (1) {
		if (try_i == 100000)
			break;
		int x = (rand() % (len - 2)) + 2;
		int y = (rand() % (len - 2)) + 2;
		if (x > y)
			swap(x, y);
		if (x == y)
			continue;

		double dist_o = dist[rt[x - 1]][rt[x]] + dist[rt[y]][rt[y + 1]];
		double dist_n = dist[rt[x - 1]][rt[y]] + dist[rt[x]][rt[y + 1]];
		double diff = dist_o - dist_n;

		if (diff > 0) {
			twoOptSwap(len, x, y, rt);
			try_i = 0;
		//	cout << opt_number++ << "번째 개선" << endl;
		//	double sum = sumPath(len, rt);
		//	cout << "Sum: " << sum << endl;
		}
		else
			try_i++;
	}
}

int remem_route[NUM_OF_NODE];
double remembest;
void Swap_ThreeOpt1(int len, int *rt, int A, int B, int C, int D, int E, int F){
	int new_route[NUM_OF_NODE];
	int k = 0;
	for (int i = 0; i <= A; i++)
		new_route[k++] = rt[i];
	for (int i = C; i >= E; i--)
		new_route[k++] = rt[i];
	for (int i = D; i <= B; i++)
		new_route[k++] = rt[i];
	for (int i = F; i <= len; i++)
		new_route[k++] = rt[i];
	for (int i = 0; i <= len; i++)
		rt[i] = new_route[i];
}

void Swap_ThreeOpt2(int len, int *rt, int A, int B, int C, int D, int E, int F){
	int new_route[NUM_OF_NODE];
	int k = 0;
	for (int i = 0; i <= A; i++)
		new_route[k++] = rt[i];
	for (int i = E; i <= C; i++)
		new_route[k++] = rt[i];
	for (int i = D; i <= B; i++)
		new_route[k++] = rt[i];
	for (int i = F; i <= len; i++)
		new_route[k++] = rt[i];
	for (int i = 0; i <= len; i++)
		rt[i] = new_route[i];
}

void Swap_ThreeOpt3(int len, int *rt, int A, int B, int C, int D, int E, int F){
	int new_route[NUM_OF_NODE];
	int k = 0;
	for (int i = 0; i <= A; i++)
		new_route[k++] = rt[i];
	for (int i = B; i >= D; i--)
		new_route[k++] = rt[i];
	for (int i = C; i >= E; i--)
		new_route[k++] = rt[i];
	for (int i = F; i <= len; i++)
		new_route[k++] = rt[i];
	for (int i = 0; i <= len; i++)
		rt[i] = new_route[i];
}

void Swap_ThreeOpt4(int len, int *rt, int A, int B, int C, int D, int E, int F){
	int new_route[NUM_OF_NODE];
	int k = 0;
	for (int i = 0; i <= A; i++)
		new_route[k++] = rt[i];
	for (int i = E; i <= C; i++)
		new_route[k++] = rt[i];
	for (int i = B; i >= D; i--)
		new_route[k++] = rt[i];
	for (int i = F; i <= len; i++)
		new_route[k++] = rt[i];
	for (int i = 0; i <= len; i++)
		rt[i] = new_route[i];
}


void three_opt(int len, int *rt){
	//cout << "3-opt start" << '\n';
	int q = 0; // change count;
	int try_i = 0;

	while (1) {
		c_end = clock();
		exe_time = (double)(c_end - c_start) / CLOCKS_PER_SEC;
		if (exe_time > THREEOPTTIME)
			break;
		if (try_i == 1000000)
			break;

		int A = (rand() % (len - 6)) + 2;
//		int B = (rand() % (len - 4)) + 2;
//		int C = (rand() % (len - 2)) + 2;
		int B = A + (rand() % 3) + 2;
		if (B + 2 >= len)
			B = B - len + 2;
		int C = B + (rand() % 3) + 2;
		if (C + 2 >= len)
			C = C - len + 2;
		
		int d[3] = { A, B, C };
		sort(d, d + 3);
		A = d[0]; B = d[1]; C = d[2];
		
		if (A == B)
			continue;
		if (B == C)
			continue;

		int D = A + 1; //a-d
		int E = B + 1;// b-e
		int F = C + 1;//f-c

		double x = dist[rt[A]][rt[D]]; //a~an의 길이
		double y = dist[rt[B]][rt[E]];//b~ bn의 길이
		double z = dist[rt[C]][rt[F]];//c~cn의 길이

		double d0 = dist[rt[A]][rt[D]] + dist[rt[B]][rt[E]] + dist[rt[C]][rt[F]]; // 현재길이
		double d1 = dist[rt[A]][rt[C]] + dist[rt[D]][rt[E]] + dist[rt[B]][rt[F]];
		double d2 = dist[rt[A]][rt[E]] + dist[rt[D]][rt[C]] + dist[rt[B]][rt[F]];
		double d3 = dist[rt[A]][rt[B]] + dist[rt[E]][rt[F]] + dist[rt[C]][rt[D]];
		double d4 = dist[rt[A]][rt[E]] + dist[rt[C]][rt[B]] + dist[rt[F]][rt[D]];

		double p0 = sumPath(len, rt);

		if (d0 > d1){
			//	cout << delta << " " << d1 << '\n';
			try_i = 0;
			Swap_ThreeOpt1(len, rt, A, B, C, D, E, F);
			q++;
		}
		else if (d0 > d2){
			try_i = 0;
			Swap_ThreeOpt2(len, rt, A, B, C, D, E, F);
			q++;
		}
		else if (d0 > d3){
			try_i = 0;
			Swap_ThreeOpt3(len, rt, A, B, C, D, E, F);
			q++;
		}
		else if (d0 > d4){
			try_i = 0;
			Swap_ThreeOpt4(len, rt, A, B, C, D, E, F);
			q++;
		}
		else {
			try_i++;
		}
		if (q == 5000)
			break;

		//if (p0 > sumPath(len, rt))
		//	cout << q << " best: " << sumPath(len, rt) << '\n';
	}
	//cout << "after 3opt Best : " << sumPath(len, rt) << ". 3-opt end." << '\n';
}

void three_opt2(int len, int *rt) {
	//cout << "3-opt start" << '\n';
	int q = 0; // change count;
	int try_i = 0;

	while (1) {
		c_end = clock();
		exe_time = (double)(c_end - c_start) / CLOCKS_PER_SEC;
		if (exe_time > THREEOPTTIME2)
			break;
		if (try_i == 10000000)
			break;

		int A = (rand() % (len - 6)) + 2;
		//		int B = (rand() % (len - 4)) + 2;
		//		int C = (rand() % (len - 2)) + 2;
		int B = A + (rand() % 3) + 2;
		if (B + 2 >= len)
			B = B - len + 2;
		int C = B + (rand() % 3) + 2;
		if (C + 2 >= len)
			C = C - len + 2;

		int d[3] = { A, B, C };
		sort(d, d + 3);
		A = d[0]; B = d[1]; C = d[2];

		if (A == B)
			continue;
		if (B == C)
			continue;

		int D = A + 1; //a-d
		int E = B + 1;// b-e
		int F = C + 1;//f-c

		double x = dist[rt[A]][rt[D]]; //a~an의 길이
		double y = dist[rt[B]][rt[E]];//b~ bn의 길이
		double z = dist[rt[C]][rt[F]];//c~cn의 길이

		double d0 = dist[rt[A]][rt[D]] + dist[rt[B]][rt[E]] + dist[rt[C]][rt[F]]; // 현재길이
		double d1 = dist[rt[A]][rt[C]] + dist[rt[D]][rt[E]] + dist[rt[B]][rt[F]];
		double d2 = dist[rt[A]][rt[E]] + dist[rt[D]][rt[C]] + dist[rt[B]][rt[F]];
		double d3 = dist[rt[A]][rt[B]] + dist[rt[E]][rt[F]] + dist[rt[C]][rt[D]];
		double d4 = dist[rt[A]][rt[E]] + dist[rt[C]][rt[B]] + dist[rt[F]][rt[D]];

		double p0 = sumPath(len, rt);

		if (d0 > d1) {
			//	cout << delta << " " << d1 << '\n';
			try_i = 0;
			Swap_ThreeOpt1(len, rt, A, B, C, D, E, F);
			q++;
		}
		else if (d0 > d2) {
			try_i = 0;
			Swap_ThreeOpt2(len, rt, A, B, C, D, E, F);
			q++;
		}
		else if (d0 > d3) {
			try_i = 0;
			Swap_ThreeOpt3(len, rt, A, B, C, D, E, F);
			q++;
		}
		else if (d0 > d4) {
			try_i = 0;
			Swap_ThreeOpt4(len, rt, A, B, C, D, E, F);
			q++;
		}
		else {
			try_i++;
		}
		if (q == 5000)
			break;

		//if (p0 > sumPath(len, rt))
		//	cout << q << " best: " << sumPath(len, rt) << '\n';
	}
	//cout << "after 3opt Best : " << sumPath(len, rt) << ". 3-opt end." << '\n';
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

int t[NUM_OF_NODE];

int main() {
	int route[NUM_OF_NODE];						//tsp path array
	cout << "edge_sort + 3-2-3opt" << endl;
	srand(time(NULL));
	int len = -1;
	readFile(&len);
	cout << len << " node TSP" << endl;

	c_start = clock();

	//calculate distance from i-node to j-node 
	int e_count = calcDist(len);
	stable_sort(e, e + e_count, cmp);

	c_end = clock();
	exe_time = (double)(c_end - c_start) / CLOCKS_PER_SEC;
	cout << "Sorting time: " << exe_time << "seconds" << endl;

	//calculate first TSP Path
	EdgeFirst(e_count, len, route);
	//double sum = sumPath(len, route);
	//cout << "first route: " << sum << endl;

	//Optimization
	three_opt(len, route);
	//sum = sumPath(len, route);
	//cout << "3-opt Best route: " << sum << endl;
	twoOpt2(len, 500000000, route);
	//sum = sumPath(len, route);
	//cout << "2opt Best route: " << sum << endl;
	//sum = sumPath(len, route);
	//twoOpt(len, route);
	three_opt2(len,route);
	//sum path
	double sum = sumPath(len, route);
	cout << "Best route: " << sum << endl;

	//print excute time
	c_end = clock();
	exe_time = (double)(c_end - c_start) / CLOCKS_PER_SEC;
	cout << "Execute time: " << exe_time << "seconds" << endl;
	
	//print PATH
	printPath(len, route);
}
