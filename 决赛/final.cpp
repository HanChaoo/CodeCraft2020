// C headers
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
// cpp headers
#include <atomic>
#include <queue>
// pthread headers
#include <pthread.h>
#include <sys/time.h>
#include <bits/stdc++.h>


#define SUBMIT
// #define TIMING


#define THREADS 12
#define BATCH_SIZE 64
#define BLOCK_SIZE (4096)
#define MAX_EDGE_CNT 2500000
#define MAX_NODE_CNT 2500000
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define HASH_MAP_SZ 2097152 // 2^21

#define MAX_PATH_LEN 10000
#define MAX_BUCKET_SIZE 100000
#define HEAP_ADJUST_INTERVAL 10000

typedef uint16_t dist_t;
#define DIST_MAX UINT16_MAX


/********** declaration **********/
void fileparser(const char *fname);
void find_scc();
void mark_link();
int calculate_time(timeval t1, timeval t2);
int cmpfunc(const void *a, const void *b);

/********** class **********/

template <typename T>
class myvector {
public:
    T *start;
    uint16_t size; //dangerous
    uint16_t capacity;
    T &operator[](const size_t &nIndex) {
        return *(start + nIndex);
    }
    void resize(const size_t &len) {
        size = len;
    }
    void inc() {
        ++size;
        if (size > capacity) {
            capacity = size + 1;
            start = (T *)realloc(start, sizeof(T) * (capacity));
        }
    }

    void push_back(const T &val) {
        inc();
        *(start + size - 1) = val;
    }
};

template <typename Key, typename Value>
class HashNode {
public:
    Key _key;
    Value _value;
    HashNode *next;

    HashNode(Key key, Value value) : _key(key), _value(value), next(nullptr) {}
    ~HashNode() {}

    HashNode &operator=(const HashNode &node) {
        _key = node._key;
        _value = node._value;
        next = node.next;
        return *this;
    }
};
template <typename Key, typename Value>
class HashMap {
public:
    HashNode<Key, Value> **table;
    uint32_t _size;

    uint32_t hash(const uint32_t &key) {
        return key;
    }

public:
    HashMap(uint32_t size) : _size(size) {
        table = new HashNode<Key, Value> *[_size];
        for (uint32_t i = 0; i < _size; i++)
            table[i] = NULL;
    }
    ~HashMap() {}

    HashNode<Key, Value> *insert(const Key &key, const Value &value) {
        uint32_t index = hash(key) % _size;
        HashNode<Key, Value> *node = new HashNode<Key, Value>(key, value);
        node->next = table[index];
        table[index] = node;
        return node;
    }
    HashNode<Key, Value> *find(const Key &key) {
        uint32_t index = hash(key) % _size;
        HashNode<Key, Value> *node = table[index];
        while (node) {
            if (node->_key == key)
                return node;
            node = node->next;
        }
        return nullptr;
    }
    Value &operator[](const Key &key) {
        uint32_t index = hash(key) % _size;
        HashNode<Key, Value> *node = table[index];
        while (node) {
            if (node->_key == key)
                return node->_value;
            node = node->next;
        }
    }
};

/********** global variables **********/
std::atomic<uint32_t> id_allocator;
HashMap<uint32_t, uint32_t> id_hash(HASH_MAP_SZ);
uint32_t edges[MAX_EDGE_CNT][3];              // 0:src, 1:dst, 2:weight
uint32_t node_cnt;
uint32_t node_cnt_total;
uint32_t avg_degree;
uint32_t max_weight;
uint8_t  weight_flag;

// tarjan variables
uint32_t dfn[MAX_NODE_CNT];
uint32_t low[MAX_NODE_CNT];
uint32_t scc[MAX_NODE_CNT];
uint32_t head;
uint32_t tarjan_stack[MAX_NODE_CNT];
uint8_t v[MAX_NODE_CNT];
uint32_t cnt;

// mark link
#define LINK_NONE 0
#define LINK_MID 1
#define LINK_END 2
uint8_t link_flag[MAX_NODE_CNT];
struct LinkInfo {
    myvector<uint32_t> pre;
    uint32_t sum;
};
LinkInfo link_info[MAX_NODE_CNT];

// struct used for graph
#pragma pack(2)
struct Nodes_0 {
    uint32_t id_weight;
};
struct Nodes_1 {
    uint32_t id;
    uint16_t weight;
};
struct Nodes_2 {
    uint32_t id;
    uint32_t weight;
};
struct Graph {
    uint32_t head;
};
Nodes_0 childs_0[MAX_EDGE_CNT];
Nodes_1 childs_1[MAX_EDGE_CNT];
Nodes_2 childs_2[MAX_EDGE_CNT];
Graph graph[MAX_NODE_CNT + 1];
uint32_t new_id[MAX_NODE_CNT];

uint32_t indegree[MAX_NODE_CNT];
uint32_t fathers[MAX_EDGE_CNT];
uint32_t temp[MAX_NODE_CNT];

Graph rgraph[MAX_NODE_CNT + 1];

inline uint32_t get_child_id(uint32_t index) {
    if (weight_flag == 0) {
        return childs_0[index].id_weight & 0x003fffff;
    }
    else if (weight_flag == 1) {
        return childs_1[index].id;
    }
    else {
        return childs_2[index].id;
    }
}

inline uint32_t get_child_weight(uint32_t index) {
    if (weight_flag == 0) {
        return (childs_0[index].id_weight >> 22) & 0x3ff;
    }
    else if (weight_flag == 1) {
        return childs_1[index].weight;
    }
    else {
        return childs_2[index].weight;
    }
}

void construct_rgraph() {
    for (int i = 0; i < node_cnt_total; ++i) {
        for (int j = graph[i].head; j < graph[i + 1].head; ++j) {
            ++indegree[get_child_id(j)];
        }
    }
    uint32_t sum = 0;
    for (int i = 0; i < node_cnt_total; ++i) {
        rgraph[i].head = sum;
        temp[i] = sum;
        sum += indegree[i];
    }
    rgraph[node_cnt_total].head = sum;
    for (int i = 0; i < node_cnt_total; ++i) {
        for (int j = graph[i].head; j < graph[i + 1].head; ++j) {
            fathers[temp[get_child_id(j)]++] = i;
        }
    }
}

// dijkstra variables
dist_t dists[THREADS][MAX_NODE_CNT];
uint8_t visited[THREADS][MAX_NODE_CNT];
#define MAX_PRE 2
struct Path {
    uint32_t num;
    uint32_t pre_num;
    uint32_t pre[MAX_PRE];
};
double Delta[THREADS][MAX_NODE_CNT];
Path path[THREADS][MAX_NODE_CNT];
std::vector<uint32_t> pre_vec[THREADS][MAX_NODE_CNT];

struct Res {
    uint32_t id;
    double cb;
    bool operator<(const Res &b) const {
        if (fabs(this->cb - b.cb) <= 1e-4) //equal
            return this->id > b.id;
        else if (this->cb < b.cb)
            return true;
        return false;
    }
};
Res res[THREADS][MAX_NODE_CNT];
struct Visited_node {
    uint32_t size;
    uint32_t nodes[MAX_NODE_CNT];
};
Visited_node visited_node[THREADS];

struct Candidate {
    uint32_t id;
    uint32_t pre;
    dist_t weight;
    bool operator<(const Candidate &b) const {
        return this->weight > b.weight;
    }
};
Candidate heap_buf[THREADS][MAX_NODE_CNT];

// Candidate heap_bucket_buf[THREADS][MAX_PATH_LEN];

struct NoWtCandidate {
    uint32_t id;
    uint32_t pre;
};

// static Candidate *p_bucket;

const uint32_t max_bucket_weight = 1 << 16;
const uint32_t max_bucket_size = 8177;

template <class T>
struct bucket_heap {
    uint8_t *storage;
    dist_t pop_pointer;
    dist_t max_index;
    uint16_t inbucket_cnt[max_bucket_weight];
    uint8_t pad[128];

    bucket_heap() {
        storage = (decltype(storage))mmap64(
            NULL, max_bucket_weight * max_bucket_size * sizeof(T), PROT_READ | PROT_WRITE,
            MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
        memset(inbucket_cnt, 0, sizeof(inbucket_cnt));
        pop_pointer = 0;
        max_index = 0;
    }

    ~bucket_heap() {
        munmap(storage, max_bucket_weight * max_bucket_size * sizeof(T));
    }

    inline bool push(const dist_t &weight, const T &node) {
        if (weight >= max_bucket_weight)
            return false;
        if (inbucket_cnt[weight] >= max_bucket_size)
            return false;
        ((T *)storage + max_bucket_size * weight)[inbucket_cnt[weight]++] = node;
        max_index = max_index >= weight ? max_index : weight;
        return true;
    }

    inline bool pop(dist_t &weight, T &node) {

        while (inbucket_cnt[pop_pointer] == 0) {
            if (pop_pointer >= max_index)
                return false;
            ++pop_pointer;
        }
        weight = pop_pointer;
        node = ((T *)storage + max_bucket_size * pop_pointer)[--inbucket_cnt[weight]];
        return true;
    }

    inline void clear() {
        pop_pointer = 0;
        max_index = 0;
    }
};

bucket_heap<Candidate> heap[THREADS];

void dijkstra(const uint32_t tid, const uint32_t src) {
    Candidate *idq = heap_buf[tid];

    double *delta = Delta[tid];
    uint32_t heap_size = 0;
    dist_t bucket_index;

    delta[src] = 0;
    dists[tid][src] = 0;
    path[tid][src].pre_num = 0;
    path[tid][src].num = 1;
    for (int i = graph[src].head; i < graph[src + 1].head; ++i) {
        dist_t new_weight = dists[tid][src] + get_child_weight(i);
        uint32_t id_tmp = get_child_id(i);
        if (new_weight <= dists[tid][id_tmp]) {
            dists[tid][id_tmp] = new_weight;
            Candidate temp;
            temp.id = id_tmp;
            temp.pre = src;
            temp.weight = new_weight;

            if (!heap[tid].push(new_weight, temp)) {
                idq[heap_size++] = temp;
                std::push_heap(idq, idq + heap_size);
            }
        }
    }
    Candidate node;
    uint32_t temp_node_cnt = 0;
    uint32_t random_num = 12345 + tid * 137;
    while (1) {
        random_num += heap_size;

        // Adjust heap to eliminate invalid nodes
        if (heap_size && random_num % HEAP_ADJUST_INTERVAL == 0) {
            for (uint32_t i = 0; i < heap_size; i++)
                if (idq[i].weight == dists[tid][idq[i].id])
                    idq[temp_node_cnt++] = idq[i];

            if (temp_node_cnt == 0)
                break;
            std::make_heap(idq, idq + temp_node_cnt);
            heap_size = temp_node_cnt;
            temp_node_cnt = 0;
        }

        //pop node from heap, first detect magic heap
        if(!heap[tid].pop(bucket_index, node)) {
            if(heap_size) {
                std::pop_heap(idq, idq + heap_size);
                node = idq[--heap_size];
            }
            else
                 break;
        }

        //handle same length shortest path
        if (visited[tid][node.id]) {
            if (node.weight == dists[tid][node.id]) {
                //same weight shortest path
                path[tid][node.id].num += path[tid][node.pre].num;
                uint32_t pre_size = path[tid][node.id].pre_num++;
                if (pre_size < MAX_PRE)
                    path[tid][node.id].pre[pre_size] = node.pre;
                else if (pre_size == MAX_PRE) {
                    pre_vec[tid][node.id].resize(0);
                    pre_vec[tid][node.id].push_back(node.pre);
                }
                else
                    pre_vec[tid][node.id].push_back(node.pre);
            }
            continue;
        }
        //fisrt visit, record path
        delta[node.id] = 0;
        visited[tid][node.id] = 1;
        visited_node[tid].nodes[visited_node[tid].size++] = node.id;
        path[tid][node.id].num = path[tid][node.pre].num;
        path[tid][node.id].pre_num = 1;
        path[tid][node.id].pre[0] = node.pre;
        //iterate all child
        for (int i = graph[node.id].head; i < graph[node.id + 1].head; ++i) {
            dist_t new_weight = dists[tid][node.id] + get_child_weight(i);
            uint32_t id_tmp = get_child_id(i);
            if (new_weight <= dists[tid][id_tmp]) {
                dists[tid][id_tmp] = new_weight;
                Candidate temp;
                temp.id = id_tmp;
                temp.pre = node.id;
                temp.weight = new_weight;
                //push
                if (!heap[tid].push(new_weight, temp)) {
                    idq[heap_size++] = temp;
                    std::push_heap(idq, idq + heap_size);
                }
            }
        }
    }
    heap[tid].clear();
}
void cal_cb(const uint32_t tid, const uint32_t src) {
    double *delta = Delta[tid];
    uint32_t size = visited_node[tid].size;
    dists[tid][src] = DIST_MAX; //clear src

    while (size) {
        uint32_t node = visited_node[tid].nodes[size - 1];
        size--;

        dists[tid][node] = DIST_MAX; //clear node
        visited[tid][node] = 0;

        double coeff = (1 + delta[node]) / path[tid][node].num;

        if (path[tid][node].pre_num <= MAX_PRE) {
            for (int i = 0; i < path[tid][node].pre_num; ++i) {
                uint32_t v = path[tid][node].pre[i];
                delta[v] += path[tid][v].num * coeff;
            }
        }
        else {
            for (int i = 0; i < MAX_PRE; ++i) {
                uint32_t v = path[tid][node].pre[i];
                delta[v] += path[tid][v].num * coeff;
            }
            for (int i = 0; i < path[tid][node].pre_num - MAX_PRE; ++i) {
                uint32_t v = pre_vec[tid][node][i];
                delta[v] += path[tid][v].num * coeff;
            }
        }
        res[tid][node].cb += delta[node];
    }
    visited_node[tid].size = 0;
}

void propagate_cb(const uint32_t tid, const uint32_t id, const uint32_t size) {
    res[tid][id].cb += size * (link_info[id].sum - 1);
    for (uint16_t i = 0; i < link_info[id].pre.size; ++i) {
        propagate_cb(tid, link_info[id].pre[i], size + 1);
    }
}

void cal_cb_link(const uint32_t tid, const uint32_t src) {
    double *delta = Delta[tid];
    uint32_t size = visited_node[tid].size;

    dists[tid][src] = DIST_MAX; //clear src
    uint32_t times = link_info[src].sum;
    uint32_t node;
    while (size) {
        node = visited_node[tid].nodes[size - 1];
        size--;
        dists[tid][node] = DIST_MAX; //clear node
        visited[tid][node] = 0;
        double coeff = (1 + delta[node]) / path[tid][node].num;
        if (path[tid][node].pre_num <= MAX_PRE) {
            for (int i = 0; i < path[tid][node].pre_num; ++i) {
                uint32_t v = path[tid][node].pre[i];
                delta[v] += path[tid][v].num * coeff;
            }
        }
        else {
            for (int i = 0; i < MAX_PRE; ++i) {
                uint32_t v = path[tid][node].pre[i];
                delta[v] += path[tid][v].num * coeff;
            }
            for (int i = 0; i < path[tid][node].pre_num - MAX_PRE; ++i) {
                uint32_t v = pre_vec[tid][node][i];
                delta[v] += path[tid][v].num * coeff;
            }
        }
        res[tid][node].cb += delta[node] * times;
    }
    //calculate cb for pre node on link
    propagate_cb(tid, src, visited_node[tid].size);
    visited_node[tid].size = 0;
}
void *work(void *args) {
    uint32_t tid = *(uint32_t *)args;
    memset(dists[tid], 0xff, node_cnt_total * sizeof(dist_t));

    while (1) {
        uint32_t job = id_allocator.fetch_add(BATCH_SIZE, std::memory_order_relaxed);
        if (job > node_cnt_total)
            break;
#ifdef TIMING
        printf("job:%d\n", job);
#endif
        for (uint32_t i = job; i < job + BATCH_SIZE && i < node_cnt_total; ++i) {
            if (link_flag[i] == LINK_NONE) {
                dijkstra(tid, i);
                cal_cb(tid, i);
            }
            else if (link_flag[i] == LINK_MID) {
                continue;
            }
            else if (link_flag[i] == LINK_END) {
                dijkstra(tid, i);
                cal_cb_link(tid, i);
            }
        }
    }
}

void output(const char *file) {
    FILE *fp = fopen(file, "w+");
    uint32_t tmp_cnt = node_cnt_total;
    std::make_heap(res[0], res[0] + tmp_cnt);
    for (uint32_t i = 0; i < 100 && i < node_cnt_total; i++) {
        fprintf(fp, "%d,%.3f\n", res[0][0].id, res[0][0].cb);
        std::pop_heap(res[0], res[0] + tmp_cnt);
        --tmp_cnt;
    }
    fclose(fp);
}

int main(int argc, char *argv[]) {
#ifdef TIMING
    timeval t1, t2;
    gettimeofday(&t1, CLOCK_REALTIME);
#endif

#ifdef SUBMIT
    fileparser("/data/test_data.txt");
#else
    fileparser(argv[1]);
#endif

#ifdef TIMING
    gettimeofday(&t2, CLOCK_REALTIME);
    printf("fileparser total: %d us\n\n", calculate_time(t1, t2));
#endif

    mark_link();

    pthread_t threads[THREADS];
    uint32_t tids[THREADS];
    for (int i = 1; i < THREADS; i++) {
        tids[i] = i;
        pthread_create(&threads[i], NULL, work, &tids[i]);
    }

    tids[0] = 0;
    work(&tids[0]);

    for (int i = 1; i < THREADS; i++) {
        pthread_join(threads[i], NULL);
    }
    for (int i = 0; i < node_cnt_total; i++) {
        for (int j = 1; j < THREADS; ++j)
            res[0][i].cb += res[j][i].cb;
    }

#ifdef TIMING
    gettimeofday(&t1, CLOCK_REALTIME);
    printf("find_cycle total: %d us\n\n", calculate_time(t2, t1));
#endif

#ifdef SUBMIT
    output("/projects/student/result.txt");
#else
    output("./result.txt");
#endif

#ifdef TIMING
    gettimeofday(&t2, CLOCK_REALTIME);
    printf("output total: %d us\n\n", calculate_time(t1, t2));
#endif
    return 0;
}

Graph graph_temp[MAX_NODE_CNT + 1];
uint32_t ori_id[MAX_NODE_CNT];
uint8_t mapped[MAX_NODE_CNT];
void remap_id() {
    uint32_t head, tail;
    uint32_t cur, odeg;
    uint32_t id_cnt = 0;
    uint32_t offset = 0;
    for (uint32_t i = 0; i < node_cnt; i++) {
        if (mapped[i])
            continue;
        mapped[i] = 1;
        new_id[i] = id_cnt;
        res[0][id_cnt].id = ori_id[i];
        ++id_cnt;
        head = tail = 0;
        temp[tail++] = i;
        while (head < tail) {
            cur = temp[head++];
            graph[new_id[cur]].head = offset;
            for (uint32_t j = graph_temp[cur].head; j < graph_temp[cur + 1].head; ++j) {
                if (!mapped[edges[j][0]]) {
                    mapped[edges[j][0]] = 1;
                    new_id[edges[j][0]] = id_cnt;
                    res[0][id_cnt].id = ori_id[edges[j][0]];
                    temp[tail++] = edges[j][0];
                    ++id_cnt;
                }
                childs_0[offset].id_weight = (new_id[edges[j][0]] | (edges[j][2] << 22));
                childs_1[offset].id = new_id[edges[j][0]];
                childs_1[offset].weight = (uint16_t)edges[j][2];
                childs_2[offset].id = new_id[edges[j][0]];
                childs_2[offset].weight = edges[j][2];
                ++offset;
            }
        }
    }
    graph[node_cnt_total].head = offset;
}

void fileparser(const char *fname) {
#ifdef TIMING
    timeval t1, t2;
    gettimeofday(&t2, CLOCK_REALTIME);
#endif
    //open source file in read only mode
    uint32_t fd = open(fname, O_RDONLY, 0666); // open source file in read only mode
    struct stat sb;
    fstat(fd, &sb);
    const uint32_t file_len = sb.st_size;
    uint32_t file_offset = 0;
    uint32_t rd_cnt = 0;
    uint32_t offset = 0;
    uint32_t arc_cnt = 0;
    do {
        rd_cnt = MIN(BLOCK_SIZE, file_len - file_offset);
        uint8_t *buf = (uint8_t *)mmap(NULL, rd_cnt, PROT_READ, MAP_SHARED, fd, file_offset);

        file_offset += BLOCK_SIZE;
        for (uint32_t pos = 0; pos < rd_cnt; ++pos) {
            if (buf[pos] == ',')
                ++offset;
            else if (buf[pos] == '\r') { //unnecessary for unix platform!
                // do nothing for carriage return
                continue;
            }
            else if (buf[pos] == '\n') {
                // end of line, output a transaction
                // swap src and dst
                if (edges[arc_cnt][2] != 0) {
                    uint32_t temp = edges[arc_cnt][0];
                    edges[arc_cnt][0] = edges[arc_cnt][1];
                    edges[arc_cnt][1] = temp;

                    max_weight = (max_weight < edges[arc_cnt][2] ? edges[arc_cnt][2] : max_weight);
                    ++arc_cnt;
                }
                else {
                    edges[arc_cnt][0] = 0;
                    edges[arc_cnt][1] = 0;
                }
                offset = 0;
            }
            else {
                edges[arc_cnt][offset] = edges[arc_cnt][offset] * 10 + (buf[pos] - '0');
            }
        }
    } while (rd_cnt == BLOCK_SIZE);
    close(fd);
    qsort(edges, arc_cnt, sizeof(uint32_t) * 3, cmpfunc);
#ifdef TIMING
    gettimeofday(&t1, CLOCK_REALTIME);
    printf("read and sort total: %d us\n\n", calculate_time(t2, t1));
#endif
    uint32_t cnt = 0;
    id_hash.insert(edges[0][1], cnt);
    ori_id[cnt] = edges[0][1];
    for (uint32_t i = 1; i < arc_cnt; ++i) {
        if (edges[i][1] != edges[i - 1][1]) {
            ++cnt;
            id_hash.insert(edges[i][1], cnt);
            ori_id[cnt] = edges[i][1];
            graph_temp[cnt].head = i;
        }
    }
    ++cnt;
    node_cnt = cnt;
    graph_temp[cnt].head = arc_cnt;
    for (uint32_t i = 0; i < arc_cnt; ++i) {
        HashNode<uint32_t, uint32_t> *p = id_hash.find(edges[i][0]);
        if (p == NULL) {
            ori_id[cnt] = edges[i][0];
            p = id_hash.insert(edges[i][0], cnt++);
        }
        edges[i][0] = p->_value;
    }
    node_cnt_total = cnt;
    avg_degree = arc_cnt / cnt;

    if (max_weight < (1 << 10)) {
        weight_flag = 0;
    }
    else if (max_weight <= UINT16_MAX) {
        weight_flag = 1;
    }
    else {  // if (max_weight < UINT32_MAX) {
        weight_flag = 2;
    }
    remap_id();

#ifdef TIMING
    gettimeofday(&t2, CLOCK_REALTIME);
    printf("construct total: %d us\n\n", calculate_time(t1, t2));
    printf("edges:%d nodes:%d\n", arc_cnt, node_cnt_total);
#endif
}

void tarjan(uint32_t now) {
    dfn[now] = low[now] = ++cnt;
    tarjan_stack[++head] = now;
    v[now] = 1;
    for (int i = graph[now].head; i < graph[now + 1].head; ++i) {
        uint32_t id_tmp = get_child_id(i);
        if (!dfn[id_tmp]) {
            tarjan(id_tmp);
            low[now] = MIN(low[now], low[id_tmp]);
        }
        else if (v[id_tmp])
            low[now] = MIN(low[now], dfn[id_tmp]);
    }

    if (dfn[now] == low[now]) {
        int cur;
        do {
            cur = tarjan_stack[head--];
            scc[cur] = now;
            v[cur] = false;
        } while (now != cur);
    }
}

void find_scc() {
    for (uint32_t i = 0; i < node_cnt_total; ++i) {
        if (dfn[i] == 0)
            tarjan(i);
    }
}

uint32_t find_link(uint32_t id) {
    uint32_t pre;
    uint32_t odeg;
    uint32_t sum = 1;

    for (uint32_t i = rgraph[id].head; i < rgraph[id + 1].head; ++i) {
        pre = fathers[i];
        odeg = graph[pre + 1].head - graph[pre].head;

        if (odeg == 1 && scc[pre] != scc[id]) {
            link_info[id].pre.push_back(pre);
            if (link_flag[pre] != LINK_NONE) {
                link_flag[pre] = LINK_MID;
                sum += link_info[pre].sum;
            }
            else {
                link_flag[pre] = LINK_MID;
                sum += find_link(pre);
            }
        }
    }
    link_info[id].sum = sum;
    return sum;
}

void mark_link() {
    find_scc();
    construct_rgraph();
    uint32_t child_num;
    uint32_t len;
    uint32_t pre;
    uint32_t cur;
    for (uint32_t i = 0; i < node_cnt_total; ++i) {
        uint32_t odeg = graph[i + 1].head - graph[i].head;
        if (link_flag[i] != LINK_NONE)
            continue;
        link_flag[i] = LINK_END;
        find_link(i);
    }
}

int calculate_time(timeval t1, timeval t2) {
    return (int)((t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec));
}

int cmpfunc(const void *a, const void *b) {
    return (*(uint64_t *)a > *(uint64_t *)b) ? 1 : -1;
}