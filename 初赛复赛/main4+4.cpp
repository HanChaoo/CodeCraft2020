// C headers
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
// cpp headers
#include <atomic>
#include <unordered_map>
// pthread headers
#include <pthread.h>
#include <sys/time.h>
#pragma pack(1)

//#define SUBMIT
#define TIMING

#define THREADS 4
#define MIN_CYCLE 3
#define MAX_CYCLE 8
#define NUM_CYCLE (MAX_CYCLE)-(MIN_CYCLE)+1
#define BATCH_SIZE 256
#define BLOCK_SIZE (4096)
#define MAX_EDGE_CNT 2000000
#define MAX_NODE_CNT 2000000
#define MAX(a, b) ((a)>(b)?(a):(b))
#define MIN(a, b) ((a)<(b)?(a):(b))
#define IS_INVALID_PATH(pre, next) \
        ((uint64_t)(pre)*1>(uint64_t)(next)*5 || (uint64_t)(next)*1>(uint64_t)(pre)*3)


/********** class **********/
class mystring {
public:
    uint8_t *ptr;
    uint32_t size;
    uint32_t capacity;

    void append(const uint8_t *src, const uint32_t len, const uint8_t end) {
        uint32_t new_size = size + len + 1;
        if (new_size > capacity) {
            capacity = new_size * 2;
            ptr = (uint8_t *) realloc(ptr, sizeof(uint8_t) * capacity);
        }
        memcpy(ptr + size, src, len);
        ptr[size + len] = end;
        size = new_size;
    }
};

template<typename T>
class myvector {
public:
    T *start;
    uint16_t size;//dangerous
    uint16_t capacity;

    // vector() = default;
    // ~vector() { free(start); }

    T &operator[](const size_t &nIndex) {
        return *(start + nIndex);
    }

    void clear() {
        size = 0;
    }

    void inc() {
        ++size;
        if (size > capacity) {
            start = (T *) realloc(start, sizeof(T) * (size));
            capacity = size;
        }
    }
};

template<typename Key, typename Value>
class HashNode {
public:
    Key    _key;
    Value  _value;
    HashNode *next;

    HashNode(Key key, Value value) : _key(key), _value(value), next(nullptr) {}
    ~HashNode() {}

    HashNode& operator=(const HashNode& node) {
        _key  = node._key;
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
    ~HashMap() {
    }

    bool insert(const Key& key, const Value& value)
    {
        uint32_t index = hash(key) % _size;
        HashNode<Key, Value> * node = new HashNode<Key, Value>(key,value);
        node->next = table[index];
        table[index] = node;
        return true;
    }
    HashNode<Key, Value>* find(const Key &key) {
        uint32_t index = hash(key) % _size;
        HashNode<Key, Value> *node = table[index];
        while (node) {
            if (node->_key == key)
                return node;
            node = node->next;
        }
        return nullptr;
    }
    Value& operator[](const Key& key) {
        uint32_t index = hash(key) % _size;
        HashNode<Key, Value> * node = table[index];
        while (node)
        {
            if (node->_key == key)
                return node->_value;
            node = node->next;
        }
    }
};


/********** global variables **********/
std::atomic<uint32_t> id_allocator;
HashMap<uint32_t, uint32_t> id_hash(2097152); // 2^21
uint32_t edges[MAX_EDGE_CNT+1][4]; // 0:src, 1:dst, 2:weight
uint32_t edges_tt[4][MAX_EDGE_CNT / 2][4];
uint32_t edges_t[2][MAX_EDGE_CNT][4];
// uint32_t edges_temp[MAX_EDGE_CNT][3];

uint8_t id_table[MAX_NODE_CNT][11]; // offset,0..0,str(id),('\n' or ',')

// struct used for graph
struct Nodes {
    uint32_t id;
    uint64_t weight;
};
struct MapHead {    // graph before topological sort
    uint32_t head;
    uint32_t tail;
    uint32_t ori_id;
    uint32_t new_id;
    std::atomic<uint32_t> indegree;
};
struct Graph {     // graph after topological sort
    uint32_t head;
    //uint32_t tail;
};
Nodes tempN[MAX_EDGE_CNT];
MapHead maphead[MAX_NODE_CNT]; // graph before topological sort
Nodes srcs[MAX_EDGE_CNT];
Nodes dsts[MAX_EDGE_CNT];
Graph graph[MAX_NODE_CNT+1];     // graph after topological sort
Graph fathers[MAX_NODE_CNT+1];   // graph after topological sort

// struct used for dfs
struct DFS_FLAG {
    uint32_t id;
    uint32_t child_offset;
    uint32_t child_num;
    uint64_t weight;
};
struct rPath3 {
    uint32_t path[3];
    uint64_t weight[2];
};
struct Colored {  // clear color(node_falgs)
    uint32_t size;
    uint32_t id[MAX_NODE_CNT];
};
DFS_FLAG dfs_stack[THREADS][16];
rPath3 r_sortbuf[THREADS][MAX_EDGE_CNT];
myvector<rPath3> path3[THREADS][MAX_NODE_CNT];
//#define PATH_BUF_SIZE 256*1024*1024
#define PATH_BUF_LEN 40000
#define PATH_BUF_WID 513
rPath3 path_buf[THREADS][PATH_BUF_LEN*PATH_BUF_WID];
uint16_t path_size[THREADS][PATH_BUF_LEN];
uint16_t path_entry[THREADS][MAX_NODE_CNT];

uint8_t node_flags[THREADS][MAX_NODE_CNT / 8];  // color
Colored colored[THREADS];

// struct used for cycles
struct Cycle {
    mystring buf[NUM_CYCLE];
    //uint32_t pad[12];
};  // 128 bytes
struct Cycles_Cnt {
    uint32_t cycle_cnt;
    uint32_t pad[63];
};  // 256 bytes
Cycle cycles[MAX_NODE_CNT / BATCH_SIZE + 1];
Cycles_Cnt cycle_thread[THREADS] = {0};
uint32_t cycle_cnt = 0;
uint32_t node_cnt = 0;

// struct used for multi-thread
struct Thread_Args {
    uint32_t thread_id;
    uint32_t start;
    uint32_t len;
};

// read file descriptor
uint32_t fd;

/********** declaration **********/
extern const char itos_lut[1000][4];

void fileparser(const char *fname);

void itos_fast_general(uint8_t *const buf, uint32_t id);

void graph_construct();

void color_node(const uint32_t root, uint32_t tid);

void *find_cycle(void *args);

void output_new(const char *file);

/********** func defination **********/
int calculate_time(timeval t1, timeval t2) {
    return (int) ((t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec));
}

int cmpfunc(const void *a, const void *b) {
    return (*(uint64_t *) a > *(uint64_t *) b) ? 1 : -1;
}
int cmpfunc1(const void *a, const void *b) {
    uint32_t *p,*q;
    p = (uint32_t *)a;
    q = (uint32_t *)b;
    if(*(uint64_t *) (p+1) > *(uint64_t *) (q+1))
        return 1;
    else if(*(uint64_t *) (p+1) == *(uint64_t *) (q+1)) {
        return (p[0] > q[0])?1:-1;
    }
    else
        return -1;
}

#define IS_COLORED(tid, id) \
        ((node_flags[(tid)][(id)>>3] & (0x1<<((id)&0x7))) != 0)
#define COLORE(tid, id) \
        (node_flags[(tid)][(id)>>3] |= (0x1<<((id)&0x7)))


bool color_node_fast(const uint32_t root, uint32_t tid) {
    rPath3 *const paths = (rPath3 *)path_buf[tid];
    rPath3 *const buf = r_sortbuf[tid];
    uint32_t offset = 0;
    for (uint32_t i = fathers[root].head; i < fathers[root+1].head; i++) {
        uint32_t next1 = srcs[i].id;
        uint64_t weight1 = srcs[i].weight;
        if (next1 > root) {
            for (uint32_t i = fathers[next1].head; i < fathers[next1+1].head; i++) {
                uint32_t next2 = srcs[i].id;
                uint64_t weight2 = srcs[i].weight;
                if (next2 > root) {
                    if (IS_INVALID_PATH(weight2, weight1))
                        continue;
                    for (uint32_t i = fathers[next2].head; i < fathers[next2+1].head; i++) {
                        uint32_t next3 = srcs[i].id;
                        uint64_t weight3 = srcs[i].weight;
                        if(next3 > root && next3 != next1) {
                            if (IS_INVALID_PATH(weight3, weight2))
                                continue;
                            buf[offset].path[0] = next1;
                            buf[offset].path[1] = next2;
                            buf[offset].path[2] = next3;
                            buf[offset].weight[0] = weight1;
                            buf[offset].weight[1] = weight3;
                            ++offset;
                        }
                    }
                }
            }
        }
    }
    qsort(buf, offset, sizeof(rPath3), cmpfunc1);
    uint16_t alloc = 0;
    for (int ct = 0; ct < offset; ct++) {
        for (uint32_t i = fathers[buf[ct].path[2]].head; i < fathers[buf[ct].path[2]+1].head; i++) {
            if (srcs[i].id > root && srcs[i].id != buf[ct].path[0] && srcs[i].id != buf[ct].path[1]) {
                if (IS_INVALID_PATH(srcs[i].weight, buf[ct].weight[1]))
                    continue;
                //add path
                uint16_t *entry = &path_entry[tid][srcs[i].id];
                if (!IS_COLORED(tid, srcs[i].id)) {
                    COLORE(tid, srcs[i].id);
                    colored[tid].id[colored[tid].size++] = srcs[i].id >> 3;
                    if(alloc >= PATH_BUF_LEN) {
                        for (int k = 0; k < colored[tid].size; k++)
                            node_flags[tid][colored[tid].id[k]] = 0;
                        colored[tid].size = 0;
                        return false;
                    }
                    path_size[tid][alloc] = 0;
                    *entry = alloc++;
                }
                //add path
                uint32_t path3_size = path_size[tid][*entry]++;
                if(path3_size >= PATH_BUF_WID) {
                    //too many path,clear colored flag
                    for (int k = 0; k < colored[tid].size; k++)
                        node_flags[tid][colored[tid].id[k]] = 0;
                    colored[tid].size = 0;
                    return false;
                }
                rPath3 *target = paths+*entry*PATH_BUF_WID+path3_size;
                target->path[0] = buf[ct].path[0];
                target->path[1] = buf[ct].path[1];
                target->path[2] = buf[ct].path[2];
                target->weight[0] = srcs[i].weight;
                target->weight[1] = buf[ct].weight[0];
            }
        }
    }
    return true;
}
void dfs_fast(uint32_t root,uint32_t tid) {
    uint32_t job_root = root/BATCH_SIZE;
    rPath3 *const paths = (rPath3 *)path_buf[tid];
    dfs_stack[tid][1].id = root;
    dfs_stack[tid][1].child_offset = graph[root].head;
    dfs_stack[tid][1].child_num = graph[root+1].head;
    level_1 :
    {
        DFS_FLAG &dfs_flag = dfs_stack[tid][1];
        if (dfs_flag.child_offset < dfs_flag.child_num) {
            uint32_t next = dsts[dfs_flag.child_offset].id;
            uint64_t weight = dsts[dfs_flag.child_offset].weight;
            dfs_flag.child_offset++;
            if (next > root) {
                dfs_stack[tid][2].id = next;
                dfs_stack[tid][2].child_offset = graph[next].head;
                dfs_stack[tid][2].child_num = graph[next+1].head;
                dfs_stack[tid][2].weight = weight;
                if (IS_COLORED(tid, next)) {
                    //cycle length=4
                    //path3
                    uint16_t entry = path_entry[tid][next];
                    for (int i = 0; i < path_size[tid][entry]; i++) {
                        rPath3 *path = paths+entry*PATH_BUF_WID+i;
                        if (!IS_INVALID_PATH(weight, path->weight[0]))
                            if (!IS_INVALID_PATH(path->weight[1], dfs_stack[tid][2].weight)) {
                                cycle_thread[tid].cycle_cnt++;
                                uint8_t *id_string;
                                for (int k = 1; k <= 2; k++) {
                                    id_string = id_table[dfs_stack[tid][k].id];
                                    cycles[job_root].buf[2].append(id_string + id_string[0], 11 - id_string[0],',');
                                }
                                id_string = id_table[path->path[2]];
                                cycles[job_root].buf[2].append(id_string + id_string[0], 11 - id_string[0],',');
                                id_string = id_table[path->path[1]];
                                cycles[job_root].buf[2].append(id_string + id_string[0], 11 - id_string[0],',');
                                id_string = id_table[path->path[0]];
                                cycles[job_root].buf[2].append(id_string + id_string[0], 11 - id_string[0],'\n');
                            }
                    }
                }
                goto level_2;
            }
            goto level_1;
        } else {
            //clear colored flag
            for (int k = 0; k < colored[tid].size; k++) {
                node_flags[tid][colored[tid].id[k]] = 0;
            }
            colored[tid].size = 0;
            return;
        }
    }
    level_2 :
    {
        DFS_FLAG &dfs_flag = dfs_stack[tid][2];
        if (dfs_flag.child_offset < dfs_flag.child_num) {
            uint32_t next = dsts[dfs_flag.child_offset].id;
            uint64_t weight = dsts[dfs_flag.child_offset].weight;
            dfs_flag.child_offset++;
            if (next > root) {
                if (IS_INVALID_PATH(dfs_stack[tid][2].weight, weight))
                    goto level_2;
                dfs_stack[tid][3].id = next;
                dfs_stack[tid][3].child_offset = graph[next].head;
                dfs_stack[tid][3].child_num = graph[next+1].head;
                dfs_stack[tid][3].weight = weight;
                if (IS_COLORED(tid, next)) {
                    //cycle length=5
                    //path3
                    uint16_t entry = path_entry[tid][next];
                    for (int i = 0; i < path_size[tid][entry]; i++) {
                        rPath3 *path = paths+entry*PATH_BUF_WID+i;
                        if (path->path[0] == dfs_stack[tid][2].id ||
                            path->path[1] == dfs_stack[tid][2].id ||
                            path->path[2] == dfs_stack[tid][2].id )
                            continue;
                        if (!IS_INVALID_PATH(weight, path->weight[0]))
                            if (!IS_INVALID_PATH(path->weight[1], dfs_stack[tid][2].weight)) {

                                cycle_thread[tid].cycle_cnt++;
                                uint8_t *id_string;
                                for (int k = 1; k <= 3; k++) {
                                    id_string = id_table[dfs_stack[tid][k].id];
                                    cycles[job_root].buf[3].append(id_string + id_string[0], 11 - id_string[0],
                                                                    ',');
                                }
                                id_string = id_table[path->path[2]];
                                cycles[job_root].buf[3].append(id_string + id_string[0], 11 - id_string[0],
                                                                ',');
                                id_string = id_table[path->path[1]];
                                cycles[job_root].buf[3].append(id_string + id_string[0], 11 - id_string[0],
                                                                ',');
                                id_string = id_table[path->path[0]];
                                cycles[job_root].buf[3].append(id_string + id_string[0], 11 - id_string[0],
                                                                '\n');
                            }
                    }
                }
                goto level_3;
            }
            goto level_2;
        } else {
            goto level_1;
        }
    }
    level_3 :
    {
        DFS_FLAG &dfs_flag = dfs_stack[tid][3];
        if (dfs_flag.child_offset < dfs_flag.child_num) {
            uint32_t next = dsts[dfs_flag.child_offset].id;
            uint64_t weight = dsts[dfs_flag.child_offset].weight;
            dfs_flag.child_offset++;
            if (next > root && next != dfs_stack[tid][2].id) {
                if (IS_INVALID_PATH(dfs_stack[tid][3].weight, weight))
                    goto level_3;
                dfs_stack[tid][4].id = next;
                dfs_stack[tid][4].child_offset = graph[next].head;
                dfs_stack[tid][4].child_num = graph[next+1].head;
                dfs_stack[tid][4].weight = weight;
                if (IS_COLORED(tid, next)) {
                    //cycle length=6
                    //path3
                    uint16_t entry = path_entry[tid][next];
                    for (int i = 0; i < path_size[tid][entry]; i++) {
                        rPath3 *path = paths+entry*PATH_BUF_WID+i;

                        if (path->path[0] == dfs_stack[tid][2].id ||
                            path->path[0] == dfs_stack[tid][3].id)
                            continue;
                        if (path->path[1] == dfs_stack[tid][2].id ||
                            path->path[1] == dfs_stack[tid][3].id)
                            continue;
                        if (path->path[2] == dfs_stack[tid][2].id ||
                            path->path[2] == dfs_stack[tid][3].id)
                            continue;
                        if (!IS_INVALID_PATH(weight, path->weight[0]))
                            if (!IS_INVALID_PATH(path->weight[1], dfs_stack[tid][2].weight)) {
                                cycle_thread[tid].cycle_cnt++;
                                uint8_t *id_string;
                                for (int k = 1; k <= 4; k++) {
                                    id_string = id_table[dfs_stack[tid][k].id];
                                    cycles[job_root].buf[4].append(id_string + id_string[0], 11 - id_string[0],
                                                                    ',');
                                }
                                id_string = id_table[path->path[2]];
                                cycles[job_root].buf[4].append(id_string + id_string[0], 11 - id_string[0],
                                                                ',');
                                id_string = id_table[path->path[1]];
                                cycles[job_root].buf[4].append(id_string + id_string[0], 11 - id_string[0],
                                                                ',');
                                id_string = id_table[path->path[0]];
                                cycles[job_root].buf[4].append(id_string + id_string[0], 11 - id_string[0],
                                                                '\n');
                            }
                    }
                }
                goto level_4;
            } else if (next == root) {
                //cycle3
                if (IS_INVALID_PATH(dfs_stack[tid][3].weight, weight))
                    goto level_3;
                if (IS_INVALID_PATH(weight, dfs_stack[tid][2].weight))
                    goto level_3;
                cycle_thread[tid].cycle_cnt++;
                uint8_t *id_string;
                for (int k = 1; k <= 2; k++) {
                    id_string = id_table[dfs_stack[tid][k].id];
                    cycles[job_root].buf[0].append(id_string + id_string[0], 11 - id_string[0], ',');
                }
                id_string = id_table[dfs_stack[tid][3].id];
                cycles[job_root].buf[0].append(id_string + id_string[0], 11 - id_string[0], '\n');
            }
            goto level_3;
        } else {
            goto level_2;
        }
    }
    level_4 :
    {
        DFS_FLAG &dfs_flag = dfs_stack[tid][4];
        if (dfs_flag.child_offset < dfs_flag.child_num) {
            uint32_t next = dsts[dfs_flag.child_offset].id;
            uint64_t weight = dsts[dfs_flag.child_offset].weight;
            dfs_flag.child_offset++;
            if (next > root && IS_COLORED(tid, next)) {
                if (next == dfs_stack[tid][2].id || next == dfs_stack[tid][3].id)
                    goto level_4;

                if (IS_INVALID_PATH(dfs_stack[tid][4].weight, weight))
                    goto level_4;

                uint16_t entry = path_entry[tid][next];
                for (int i = 0; i < path_size[tid][entry]; i++) {
                    rPath3 *path = paths+entry*PATH_BUF_WID+i;
                    if (path->path[0] == dfs_stack[tid][2].id ||
                        path->path[0] == dfs_stack[tid][3].id ||
                        path->path[0] == dfs_stack[tid][4].id)
                        continue;
                    if (path->path[1] == dfs_stack[tid][2].id ||
                        path->path[1] == dfs_stack[tid][3].id ||
                        path->path[1] == dfs_stack[tid][4].id)
                        continue;
                    if (path->path[2] == dfs_stack[tid][2].id ||
                        path->path[2] == dfs_stack[tid][3].id ||
                        path->path[2] == dfs_stack[tid][4].id)
                        continue;
                    if (!IS_INVALID_PATH(weight, path->weight[0]))
                        if (!IS_INVALID_PATH(path->weight[1], dfs_stack[tid][2].weight)) {
                            cycle_thread[tid].cycle_cnt++;
                            uint8_t *id_string;
                            for (int k = 1; k <= 4; k++) {
                                id_string = id_table[dfs_stack[tid][k].id];
                                cycles[job_root].buf[5].append(id_string + id_string[0], 11 - id_string[0],
                                                                ',');
                            }
                            id_string = id_table[next];
                            cycles[job_root].buf[5].append(id_string + id_string[0], 11 - id_string[0], ',');
                            id_string = id_table[path->path[2]];
                            cycles[job_root].buf[5].append(id_string + id_string[0], 11 - id_string[0], ',');
                            id_string = id_table[path->path[1]];
                            cycles[job_root].buf[5].append(id_string + id_string[0], 11 - id_string[0], ',');
                            id_string = id_table[path->path[0]];
                            cycles[job_root].buf[5].append(id_string + id_string[0], 11 - id_string[0], '\n');
                        }
                }
            }
            else if(next == root) {
                if (IS_INVALID_PATH(dfs_stack[tid][4].weight, weight))
                    goto level_4;
                if (IS_INVALID_PATH(weight, dfs_stack[tid][2].weight))
                    goto level_4;
                cycle_thread[tid].cycle_cnt++;
                uint8_t *id_string;
                for (int k = 1; k <= 3; k++) {
                    id_string = id_table[dfs_stack[tid][k].id];
                    cycles[job_root].buf[1].append(id_string + id_string[0], 11 - id_string[0], ',');
                }
                id_string = id_table[dfs_stack[tid][4].id];
                cycles[job_root].buf[1].append(id_string + id_string[0], 11 - id_string[0], '\n');
            }
            goto level_4;
        } else {
            goto level_3;
        }
    }
}
void color_node(const uint32_t root, uint32_t tid) {
    rPath3 *const buf = r_sortbuf[tid];
    uint32_t offset = 0;
    for (uint32_t i = fathers[root].head; i < fathers[root+1].head; i++) {
        uint32_t next1 = srcs[i].id;
        uint64_t weight1 = srcs[i].weight;
        if (next1 > root) {
            for (uint32_t i = fathers[next1].head; i < fathers[next1+1].head; i++) {
                uint32_t next2 = srcs[i].id;
                uint64_t weight2 = srcs[i].weight;
                if (next2 > root) {
                    if (IS_INVALID_PATH(weight2, weight1))
                        continue;
                    for (uint32_t i = fathers[next2].head; i < fathers[next2+1].head; i++) {
                        uint32_t next3 = srcs[i].id;
                        uint64_t weight3 = srcs[i].weight;
                        if(next3 > root && next3 != next1) {
                            if (IS_INVALID_PATH(weight3, weight2))
                                continue;
                            buf[offset].path[0] = next1;
                            buf[offset].path[1] = next2;
                            buf[offset].path[2] = next3;
                            buf[offset].weight[0] = weight1;
                            buf[offset].weight[1] = weight3;
                            ++offset;
                        }
                    }
                }
            }
        }
    }
    qsort(buf, offset, sizeof(rPath3), cmpfunc1);
    for (int ct = 0; ct < offset; ct++) {
        for (uint32_t i = fathers[buf[ct].path[2]].head; i < fathers[buf[ct].path[2]+1].head; i++) {
            if (srcs[i].id > root && srcs[i].id != buf[ct].path[0] && srcs[i].id != buf[ct].path[1]) {
                if (IS_INVALID_PATH(srcs[i].weight, buf[ct].weight[1]))
                    continue;
                if (!IS_COLORED(tid, srcs[i].id)) {
                    COLORE(tid, srcs[i].id);
                    colored[tid].id[colored[tid].size++] = srcs[i].id >> 3;
                    path3[tid][srcs[i].id].clear();
                }
                //add path
                uint32_t path3_size = path3[tid][srcs[i].id].size;
                path3[tid][srcs[i].id].inc();
                path3[tid][srcs[i].id][path3_size].path[0] = buf[ct].path[0];
                path3[tid][srcs[i].id][path3_size].path[1] = buf[ct].path[1];
                path3[tid][srcs[i].id][path3_size].path[2] = buf[ct].path[2];
                path3[tid][srcs[i].id][path3_size].weight[0] = srcs[i].weight;
                path3[tid][srcs[i].id][path3_size].weight[1] = buf[ct].weight[0];
            }
        }
    }
}
void dfs(uint32_t root,uint32_t tid) {
    uint32_t job_root = root/BATCH_SIZE;
    dfs_stack[tid][1].id = root;
    dfs_stack[tid][1].child_offset = graph[root].head;
    dfs_stack[tid][1].child_num = graph[root+1].head;
    level_1 :
    {
        DFS_FLAG &dfs_flag = dfs_stack[tid][1];
        if (dfs_flag.child_offset < dfs_flag.child_num) {
            uint32_t next = dsts[dfs_flag.child_offset].id;
            uint64_t weight = dsts[dfs_flag.child_offset].weight;
            dfs_flag.child_offset++;
            if (next > root) {
                //printf()
                dfs_stack[tid][2].id = next;
                dfs_stack[tid][2].child_offset = graph[next].head;
                dfs_stack[tid][2].child_num = graph[next+1].head;
                dfs_stack[tid][2].weight = weight;
                if (IS_COLORED(tid, next)) {
                    //cycle length=4
                    //path3

                    for (int i = 0; i < path3[tid][next].size; i++) {
                        if (!IS_INVALID_PATH(weight, path3[tid][next][i].weight[0]))
                            if (!IS_INVALID_PATH(path3[tid][next][i].weight[1], dfs_stack[tid][2].weight)) {
                                cycle_thread[tid].cycle_cnt++;
                                uint8_t *id_string;
                                for (int k = 1; k <= 2; k++) {
                                    id_string = id_table[dfs_stack[tid][k].id];
                                    cycles[job_root].buf[2].append(id_string + id_string[0], 11 - id_string[0],
                                                                    ',');
                                }
                                id_string = id_table[path3[tid][next][i].path[2]];
                                cycles[job_root].buf[2].append(id_string + id_string[0], 11 - id_string[0],
                                                                ',');
                                id_string = id_table[path3[tid][next][i].path[1]];
                                cycles[job_root].buf[2].append(id_string + id_string[0], 11 - id_string[0],
                                                                ',');
                                id_string = id_table[path3[tid][next][i].path[0]];
                                cycles[job_root].buf[2].append(id_string + id_string[0], 11 - id_string[0],
                                                                '\n');
                            }
                    }
                }
                goto level_2;
            }
            goto level_1;
        } else {
            //clear colored flag
            for (int k = 0; k < colored[tid].size; k++) {
                node_flags[tid][colored[tid].id[k]] = 0;
            }
            colored[tid].size = 0;
            return;
        }
    }
    level_2 :
    {
        DFS_FLAG &dfs_flag = dfs_stack[tid][2];
        if (dfs_flag.child_offset < dfs_flag.child_num) {
            uint32_t next = dsts[dfs_flag.child_offset].id;
            uint64_t weight = dsts[dfs_flag.child_offset].weight;
            dfs_flag.child_offset++;
            if (next > root) {
                if (IS_INVALID_PATH(dfs_stack[tid][2].weight, weight))
                    goto level_2;
                dfs_stack[tid][3].id = next;
                dfs_stack[tid][3].child_offset = graph[next].head;
                dfs_stack[tid][3].child_num = graph[next+1].head;
                dfs_stack[tid][3].weight = weight;
                if (IS_COLORED(tid, next)) {
                    //cycle length=5
                    //path3
                    for (int i = 0; i < path3[tid][next].size; i++) {
                        if (path3[tid][next][i].path[0] == dfs_stack[tid][2].id ||
                            path3[tid][next][i].path[1] == dfs_stack[tid][2].id ||
                            path3[tid][next][i].path[2] == dfs_stack[tid][2].id )
                            continue;

                        if (!IS_INVALID_PATH(weight, path3[tid][next][i].weight[0]))
                            if (!IS_INVALID_PATH(path3[tid][next][i].weight[1], dfs_stack[tid][2].weight)) {

                                cycle_thread[tid].cycle_cnt++;
                                uint8_t *id_string;
                                for (int k = 1; k <= 3; k++) {
                                    id_string = id_table[dfs_stack[tid][k].id];
                                    cycles[job_root].buf[3].append(id_string + id_string[0], 11 - id_string[0],
                                                                    ',');
                                }
                                id_string = id_table[path3[tid][next][i].path[2]];
                                cycles[job_root].buf[3].append(id_string + id_string[0], 11 - id_string[0],
                                                                ',');
                                id_string = id_table[path3[tid][next][i].path[1]];
                                cycles[job_root].buf[3].append(id_string + id_string[0], 11 - id_string[0],
                                                                ',');
                                id_string = id_table[path3[tid][next][i].path[0]];
                                cycles[job_root].buf[3].append(id_string + id_string[0], 11 - id_string[0],
                                                                '\n');
                            }
                    }
                }
                goto level_3;
            }
            goto level_2;
        } else {
            goto level_1;
        }
    }
    level_3 :
    {
        DFS_FLAG &dfs_flag = dfs_stack[tid][3];
        if (dfs_flag.child_offset < dfs_flag.child_num) {
            uint32_t next = dsts[dfs_flag.child_offset].id;
            uint64_t weight = dsts[dfs_flag.child_offset].weight;
            dfs_flag.child_offset++;
            if (next > root && next != dfs_stack[tid][2].id) {
                if (IS_INVALID_PATH(dfs_stack[tid][3].weight, weight))
                    goto level_3;
                dfs_stack[tid][4].id = next;
                dfs_stack[tid][4].child_offset = graph[next].head;
                dfs_stack[tid][4].child_num = graph[next+1].head;
                dfs_stack[tid][4].weight = weight;
                if (IS_COLORED(tid, next)) {
                    //cycle length=6
                    //path3
                    for (int i = 0; i < path3[tid][next].size; i++) {
                        if (path3[tid][next][i].path[0] == dfs_stack[tid][2].id ||
                            path3[tid][next][i].path[0] == dfs_stack[tid][3].id )
                                continue;
                        if (path3[tid][next][i].path[1] == dfs_stack[tid][2].id ||
                            path3[tid][next][i].path[1] == dfs_stack[tid][3].id)
                            continue;
                        if (path3[tid][next][i].path[2] == dfs_stack[tid][2].id ||
                            path3[tid][next][i].path[2] == dfs_stack[tid][3].id)
                            continue;

                        if (!IS_INVALID_PATH(weight, path3[tid][next][i].weight[0]))
                            if (!IS_INVALID_PATH(path3[tid][next][i].weight[1], dfs_stack[tid][2].weight)) {
                                cycle_thread[tid].cycle_cnt++;
                                uint8_t *id_string;
                                for (int k = 1; k <= 4; k++) {
                                    id_string = id_table[dfs_stack[tid][k].id];
                                    cycles[job_root].buf[4].append(id_string + id_string[0], 11 - id_string[0],
                                                                    ',');
                                }
                                id_string = id_table[path3[tid][next][i].path[2]];
                                cycles[job_root].buf[4].append(id_string + id_string[0], 11 - id_string[0],
                                                                ',');
                                id_string = id_table[path3[tid][next][i].path[1]];
                                cycles[job_root].buf[4].append(id_string + id_string[0], 11 - id_string[0],
                                                                ',');
                                id_string = id_table[path3[tid][next][i].path[0]];
                                cycles[job_root].buf[4].append(id_string + id_string[0], 11 - id_string[0],
                                                                '\n');
                            }
                    }
                }
                goto level_4;
            } else if (next == root) {
                //cycle3
                if (IS_INVALID_PATH(dfs_stack[tid][3].weight, weight))
                    goto level_3;
                if (IS_INVALID_PATH(weight, dfs_stack[tid][2].weight))
                    goto level_3;
                cycle_thread[tid].cycle_cnt++;
                uint8_t *id_string;
                for (int k = 1; k <= 2; k++) {
                    id_string = id_table[dfs_stack[tid][k].id];
                    cycles[job_root].buf[0].append(id_string + id_string[0], 11 - id_string[0], ',');
                }
                id_string = id_table[dfs_stack[tid][3].id];
                cycles[job_root].buf[0].append(id_string + id_string[0], 11 - id_string[0], '\n');
            }
            goto level_3;
        } else {
            goto level_2;
        }
    }
    level_4 :
    {
        DFS_FLAG &dfs_flag = dfs_stack[tid][4];
        if (dfs_flag.child_offset < dfs_flag.child_num) {
            uint32_t next = dsts[dfs_flag.child_offset].id;
            uint64_t weight = dsts[dfs_flag.child_offset].weight;
            dfs_flag.child_offset++;
            if (next > root && IS_COLORED(tid, next)) {
                if (next == dfs_stack[tid][2].id || next == dfs_stack[tid][3].id)
                    goto level_4;

                if (IS_INVALID_PATH(dfs_stack[tid][4].weight, weight))
                    goto level_4;
                //add all cycle len >= 5
                //path3
                for (int i = 0; i < path3[tid][next].size; i++) {

                    if (path3[tid][next][i].path[0] == dfs_stack[tid][2].id ||
                        path3[tid][next][i].path[0] == dfs_stack[tid][3].id ||
                        path3[tid][next][i].path[0] == dfs_stack[tid][4].id)
                        continue;
                    if (path3[tid][next][i].path[1] == dfs_stack[tid][2].id ||
                        path3[tid][next][i].path[1] == dfs_stack[tid][3].id ||
                        path3[tid][next][i].path[1] == dfs_stack[tid][4].id)
                        continue;
                    if (path3[tid][next][i].path[2] == dfs_stack[tid][2].id ||
                        path3[tid][next][i].path[2] == dfs_stack[tid][3].id ||
                        path3[tid][next][i].path[2] == dfs_stack[tid][4].id)
                        continue;
                    if (!IS_INVALID_PATH(weight, path3[tid][next][i].weight[0]))
                        if (!IS_INVALID_PATH(path3[tid][next][i].weight[1], dfs_stack[tid][2].weight)) {
                            cycle_thread[tid].cycle_cnt++;
                            uint8_t *id_string;
                            for (int k = 1; k <= 4; k++) {
                                id_string = id_table[dfs_stack[tid][k].id];
                                cycles[job_root].buf[5].append(id_string + id_string[0], 11 - id_string[0],
                                                                ',');
                            }
                            id_string = id_table[next];
                            cycles[job_root].buf[5].append(id_string + id_string[0], 11 - id_string[0], ',');
                            id_string = id_table[path3[tid][next][i].path[2]];
                            cycles[job_root].buf[5].append(id_string + id_string[0], 11 - id_string[0], ',');
                            id_string = id_table[path3[tid][next][i].path[1]];
                            cycles[job_root].buf[5].append(id_string + id_string[0], 11 - id_string[0], ',');
                            id_string = id_table[path3[tid][next][i].path[0]];
                            cycles[job_root].buf[5].append(id_string + id_string[0], 11 - id_string[0], '\n');
                        }
                }
            }
            else if(next == root) {

                if (IS_INVALID_PATH(dfs_stack[tid][4].weight, weight))
                    goto level_4;
                if (IS_INVALID_PATH(weight, dfs_stack[tid][2].weight))
                    goto level_4;
                cycle_thread[tid].cycle_cnt++;
                uint8_t *id_string;
                for (int k = 1; k <= 3; k++) {
                    id_string = id_table[dfs_stack[tid][k].id];
                    cycles[job_root].buf[1].append(id_string + id_string[0], 11 - id_string[0], ',');
                }
                id_string = id_table[dfs_stack[tid][4].id];
                cycles[job_root].buf[1].append(id_string + id_string[0], 11 - id_string[0], '\n');
            }
            goto level_4;
        } else {
            goto level_3;
        }
    }
}
void *find_cycle(void *args) {
#ifdef TIMING
    timeval t1, t2;
    gettimeofday(&t1, CLOCK_REALTIME);
#endif
    Thread_Args thread_args = *((Thread_Args *) (args));
    uint32_t tid = thread_args.thread_id;
    bool flag = true;
    while (flag) {
        uint32_t root = id_allocator.fetch_add(BATCH_SIZE, std::memory_order_relaxed);
        uint32_t job_root = root / BATCH_SIZE;
        for (int k = 0; k < BATCH_SIZE; k++, root++) {
            if (root >= node_cnt) {
                flag = false;
                break;
            }
             if(color_node_fast(root,tid)) {
                 dfs_fast(root,tid);
             }
             else {
                color_node(root, tid);
                dfs(root,tid);
           }

        }
    }
#ifdef TIMING
    gettimeofday(&t2, CLOCK_REALTIME);
    printf("Thread%d: %d us\n\n",tid, calculate_time(t1, t2));
#endif
}

struct Cycles_div {
    int fd;
    uint8_t *buf;
    uint32_t offset[NUM_CYCLE];
    uint32_t size[NUM_CYCLE];
    uint32_t start[NUM_CYCLE];
    uint32_t end[NUM_CYCLE];
};
void *output_thread(void *args) {
    Cycles_div *info = (Cycles_div *) args;
    for (int i = 0; i < NUM_CYCLE; ++i) {
        if (info->size[i] == 0)
            continue;
        uint32_t offset = info->offset[i];
        for (uint32_t j = info->start[i]; j < info->end[i]; ++j) {
            memcpy(info->buf + offset, cycles[j].buf[i].ptr, cycles[j].buf[i].size);
            offset += cycles[j].buf[i].size;
        }
    }
}
void output_mm(const char *file) {

    uint8_t buf[12];
    itos_fast_general(buf, cycle_cnt);
    buf[11] = '\n';
    uint32_t cycles_bytes[NUM_CYCLE] = {0};
    for (int i = 0; i < node_cnt / BATCH_SIZE + 1; ++i) {
        for (int j = 0; j < NUM_CYCLE; ++j)
            cycles_bytes[j] += cycles[i].buf[j].size;
    }
    //trunc file size
    uint32_t total_bytes = 12 - buf[0];
    for (int j = 0; j < NUM_CYCLE; ++j)
        total_bytes += cycles_bytes[j];

    int fd = open(file, O_RDWR | O_CREAT, 0666);
    ftruncate(fd, total_bytes);
    uint8_t *output_buf = (uint8_t *) mmap(NULL, total_bytes, PROT_WRITE, MAP_SHARED , fd, 0);
    memcpy(output_buf, buf + buf[0], 12 - buf[0]);
    //calculate thread divide info
    Cycles_div cycles_div[4] = {0};
    for(int i=0; i<4 ; i++) {
        cycles_div[i].fd = fd;
        cycles_div[i].buf = output_buf;
    }
    uint32_t bytes = 12 - buf[0];
    for (int i = 0; i < NUM_CYCLE; i++) {
        uint32_t size = 0;
        uint32_t t = 0;
        cycles_div[0].start[i] = 0;
        cycles_div[0].offset[i] = bytes;
        for (int j = 0; j < node_cnt / BATCH_SIZE + 1 && t < 3; ++j) {
            if (size + cycles[j].buf[i].size > cycles_bytes[i] / 4) {
                cycles_div[t + 1].start[i] = j;
                cycles_div[t + 1].offset[i] = bytes + size;
                cycles_div[t].end[i] = j;
                cycles_div[t].size[i] = size;
                bytes += size;
                size = cycles[j].buf[i].size;
                ++t;
            } else
                size += cycles[j].buf[i].size;
        }
        cycles_div[t].end[i] = node_cnt / BATCH_SIZE + 1;
        cycles_div[t].size[i] = cycles_bytes[i];
        for (int k = 0; k < t; k++)
            cycles_div[t].size[i] -= cycles_div[k].size[i];
        bytes += cycles_div[t].size[i];
    }
    pthread_t pthread[4];
    for (int i = 1; i < 4; ++i)
        pthread_create(&pthread[i], NULL, output_thread, &cycles_div[i]);
    output_thread(&cycles_div[0]);
    for (int i = 1; i < 4; ++i)
        pthread_join(pthread[i], NULL);
    close(fd);
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

    pthread_t pthread[THREADS];
    Thread_Args thread_args[THREADS];

    thread_args[0].thread_id = 0;
    // find_cycle thread
    // find_cycle((void *) &thread_args[0]);
    for (int i = 1; i < THREADS; ++i) {
        thread_args[i].thread_id = i;
        pthread_create(&pthread[i], NULL, find_cycle, &thread_args[i]);
    }

    find_cycle((void *) &thread_args[0]);

    for (int i = 1; i < THREADS; ++i) {
        pthread_join(pthread[i], NULL);
    }

    for (int i = 0; i < THREADS; ++i) {
        cycle_cnt += cycle_thread[i].cycle_cnt;
    }
    printf("%d\n",cycle_cnt);
#ifdef TIMING
    gettimeofday(&t1, CLOCK_REALTIME);
    printf("find_cycle total: %d us\n\n", calculate_time(t2, t1));
#endif


#ifdef SUBMIT
    output_mm("/projects/student/result.txt");
#else
    output_mm("./result.txt");
#endif

#ifdef TIMING
    gettimeofday(&t2, CLOCK_REALTIME);
    printf("output total: %d us\n\n", calculate_time(t1, t2));
#endif


    return 0;
}

struct Read_Args {
    uint32_t thread_id;
    uint32_t mmap_start;
    uint32_t mmap_len;
    uint32_t edge_cnt;
    uint32_t pad[124];
};

std::atomic<bool> sort_flag[2];

void *filereader(void *args) {

#ifdef TIMING
    timeval t1, t2;
    gettimeofday(&t1, CLOCK_REALTIME);
#endif

    Read_Args *t = (Read_Args *) args;
    uint32_t thread_id = t->thread_id;
    uint32_t mmap_start = t->mmap_start;
    uint32_t mmap_len = t->mmap_len;

    uint32_t line_offset = 0, pos = 0, float_digit = 0;

    uint8_t *buf;

    buf = (uint8_t *) mmap(NULL, mmap_len, PROT_READ, MAP_SHARED, fd, mmap_start);

    if (mmap_start != 0) {
        for (pos = 0; pos < mmap_len; ++pos) {
            if (buf[pos] == '\n') {
                break;
            }
        }
        ++pos;
    }

    uint32_t edge_cnt = 0;

    for (; pos < mmap_len; ++pos) {
        if (buf[pos] == ',' || buf[pos] == '.')
            ++line_offset;
        else if (buf[pos] == '\r')//unnecessary for unix platform!
            //do nothing for carriage return
            continue;
        else if (buf[pos] == '\n') {
            //end of line, output a transaction
            //swap src and dst
            uint32_t temp = edges_tt[thread_id][edge_cnt][0];
            edges_tt[thread_id][edge_cnt][0] = edges_tt[thread_id][edge_cnt][1];
            edges_tt[thread_id][edge_cnt][1] = temp;

            if (float_digit == 1)
                edges_tt[thread_id][edge_cnt][3] *= 10;

            ++edge_cnt;
            // edge_cnt = arc_cnt->fetch_add(1, std::memory_order_relaxed);

            line_offset = 0;
            float_digit = 0;
        } else {
            edges_tt[thread_id][edge_cnt][line_offset] =
                    edges_tt[thread_id][edge_cnt][line_offset] * 10 + (buf[pos] & 0x0f);
            if (line_offset == 3)
                ++float_digit;
        }
    }
    munmap(buf, mmap_len);
    buf = (uint8_t *) mmap(NULL, BLOCK_SIZE, PROT_READ, MAP_SHARED, fd, mmap_start + mmap_len);
    for (pos = 0; pos < BLOCK_SIZE; pos = pos + 1) {
        if (buf[pos] == ',' || buf[pos] == '.')
            ++line_offset;
        else if (buf[pos] == '\r') {//unnecessary for unix platform!
            //do nothing for carriage return
            continue;
        } else if (buf[pos] == '\n') {
            //end of line, output a transaction
            //swap src and dst
            uint32_t temp = edges_tt[thread_id][edge_cnt][0];
            edges_tt[thread_id][edge_cnt][0] = edges_tt[thread_id][edge_cnt][1];
            edges_tt[thread_id][edge_cnt][1] = temp;

            if (float_digit == 1)
                edges_tt[thread_id][edge_cnt][3] *= 10;

            ++edge_cnt;
            line_offset = 0;
            float_digit = 0;
            break;

        } else {
            edges_tt[thread_id][edge_cnt][line_offset] =
                    edges_tt[thread_id][edge_cnt][line_offset] * 10 + (buf[pos] & 0x0f);
            if (line_offset == 3)
                ++float_digit;
        }
    }
    munmap(buf, BLOCK_SIZE);
    t->edge_cnt = edge_cnt;

#ifdef TIMING
    gettimeofday(&t2, CLOCK_REALTIME);
    printf("thread %d readtime %d us\n", thread_id, calculate_time(t1, t2));
#endif

#ifndef SUBMIT
    printf("thread %d edges %d \n", thread_id, edge_cnt);
#endif

    qsort(edges_tt[thread_id], edge_cnt, sizeof(uint32_t) * 4, cmpfunc);

    sort_flag[thread_id / 2] = true;

#ifdef TIMING
    gettimeofday(&t1, CLOCK_REALTIME);
    printf("thread %d sorttime %d us\n", thread_id, calculate_time(t2, t1));
#endif

}

void *filereader_merge(void *args) {

#ifdef TIMING
    timeval t1, t2;
    gettimeofday(&t1, CLOCK_REALTIME);
#endif

    Read_Args *t = (Read_Args *) args;
    uint32_t thread_id = t->thread_id;
    uint32_t mmap_start = t->mmap_start;
    uint32_t mmap_len = t->mmap_len;

    uint32_t line_offset = 0, pos = 0, float_digit = 0;

    uint8_t *buf;

    buf = (uint8_t *) mmap(NULL, mmap_len, PROT_READ, MAP_SHARED, fd, mmap_start);

    if (mmap_start != 0) {
        for (pos = 0; pos < mmap_len; ++pos) {
            if (buf[pos] == '\n') {
                break;
            }
        }
        ++pos;
    }

    uint32_t edge_cnt = 0;

    for (; pos < mmap_len; ++pos) {
        if (buf[pos] == ',' || buf[pos] == '.')
            ++line_offset;
        else if (buf[pos] == '\r')//unnecessary for unix platform!
            //do nothing for carriage return
            continue;
        else if (buf[pos] == '\n') {
            //end of line, output a transaction
            //swap src and dst
            uint32_t temp = edges_tt[thread_id][edge_cnt][0];
            edges_tt[thread_id][edge_cnt][0] = edges_tt[thread_id][edge_cnt][1];
            edges_tt[thread_id][edge_cnt][1] = temp;

            if (float_digit == 1)
                edges_tt[thread_id][edge_cnt][3] *= 10;

            ++edge_cnt;

            line_offset = 0;
            float_digit = 0;
        } else {
            edges_tt[thread_id][edge_cnt][line_offset] =
                    edges_tt[thread_id][edge_cnt][line_offset] * 10 + (buf[pos] & 0x0f);
            if (line_offset == 3)
                ++float_digit;
        }
    }
    munmap(buf, mmap_len);
    if (thread_id == 1) {
        buf = (uint8_t *) mmap(NULL, BLOCK_SIZE, PROT_READ, MAP_SHARED, fd, mmap_start + mmap_len);
        for (pos = 0; pos < BLOCK_SIZE; pos = pos + 1) {
            if (buf[pos] == ',' || buf[pos] == '.')
                ++line_offset;
            else if (buf[pos] == '\r') {//unnecessary for unix platform!
                //do nothing for carriage return
                continue;
            } else if (buf[pos] == '\n') {
                //end of line, output a transaction
                //swap src and dst
                uint32_t temp = edges_tt[thread_id][edge_cnt][0];
                edges_tt[thread_id][edge_cnt][0] = edges_tt[thread_id][edge_cnt][1];
                edges_tt[thread_id][edge_cnt][1] = temp;

                if (float_digit == 1)
                    edges_tt[thread_id][edge_cnt][3] *= 10;

                ++edge_cnt;
                line_offset = 0;
                float_digit = 0;
                break;

            } else {
                edges_tt[thread_id][edge_cnt][line_offset] =
                        edges_tt[thread_id][edge_cnt][line_offset] * 10 + (buf[pos] & 0x0f);
                if (line_offset == 3)
                    ++float_digit;
            }
        }
        munmap(buf, BLOCK_SIZE);
    }

    t->edge_cnt = edge_cnt;

#ifndef SUBMIT
    printf("thread %d edges %d\n", thread_id, edge_cnt);
#endif

#ifdef TIMING
    gettimeofday(&t2, CLOCK_REALTIME);
    printf("thread %d readtime %d us\n", thread_id, calculate_time(t1, t2));
#endif

    qsort(edges_tt[thread_id], edge_cnt, sizeof(uint32_t) * 4, cmpfunc);

#ifdef TIMING
    gettimeofday(&t1, CLOCK_REALTIME);
    printf("thread %d sorttime %d us\n", thread_id, calculate_time(t2, t1));
#endif

    while (!sort_flag[thread_id / 2]);

    uint32_t *dst = (uint32_t *) (edges_t[thread_id / 2]);
    uint32_t *src0 = (uint32_t *) (edges_tt[thread_id - 1]);
    uint32_t *src1 = (uint32_t *) (edges_tt[thread_id]);
    uint32_t *end0 = (uint32_t *) (src0 + t[-1].edge_cnt * 4);
    uint32_t *end1 = (uint32_t *) (src1 + t[0].edge_cnt * 4);
    while (src0 < end0 && src1 < end1) {
        if (*(uint64_t *) src0 < *(uint64_t *) src1) {
            dst[0] = src0[0];
            dst[1] = src0[1];
            dst[2] = src0[2];
            dst[3] = src0[3];
            src0 += 4;
        } else {
            dst[0] = src1[0];
            dst[1] = src1[1];
            dst[2] = src1[2];
            dst[3] = src1[3];
            src1 += 4;
        }
        dst += 4;
    }
    while (src0 < end0) {
        dst[0] = src0[0];
        dst[1] = src0[1];
        dst[2] = src0[2];
        dst[3] = src0[3];
        src0 += 4;
        dst += 4;
    }
    while (src1 < end1) {
        dst[0] = src1[0];
        dst[1] = src1[1];
        dst[2] = src1[2];
        dst[3] = src1[3];
        src1 += 4;
        dst += 4;
    }

#ifdef TIMING
    gettimeofday(&t1, CLOCK_REALTIME);
    printf("thread %d mergetime %d us\n", thread_id, calculate_time(t2, t1));
#endif

}

void *construct_thread(void *args) {
    uint32_t *divid = (uint32_t *) args;
    uint32_t i = divid[0];
    //printf("%d %d\n",divid[0],divid[1]);
    if (i > 0)
        while (edges[i][1] == edges[i - 1][1]) ++i;
    for (; i < divid[1]; ++i) {
        if (edges[i][2] == 0)
            continue;
        auto ite = id_hash.find(edges[i][0]);
        if (ite) {
            uint32_t v = ite->_value;
            uint32_t offset = maphead[edges[i][1]].tail++;
            tempN[offset].id = v;
            tempN[offset].weight = ((uint64_t) edges[i][2] * 100) + edges[i][3];
            // tempN[offset].weight = edges[i][2];
            maphead[v].indegree.fetch_add(1, std::memory_order_relaxed);
        }
    }
    while (edges[i][1] == edges[i - 1][1]) {
        if (edges[i][2] == 0) {
            ++i;
            continue;
        }
        auto ite = id_hash.find(edges[i][0]);
        if (ite) {
            uint32_t v = ite->_value;
            uint32_t offset = maphead[edges[i][1]].tail++;
            tempN[offset].id = v;
            tempN[offset].weight = ((uint64_t) edges[i][2] * 100) + edges[i][3];
            // tempN[offset].weight = edges[i][2];
            maphead[v].indegree.fetch_add(1, std::memory_order_relaxed);
        }
        ++i;
    }
}

void fileparser(const char *fname) {
#ifdef TIMING
    timeval t1, t2;
    gettimeofday(&t2, CLOCK_REALTIME);
#endif
    //open source file in read only mode
    fd = open(fname, O_RDONLY, 0666);
    struct stat sb;
    fstat(fd, &sb);
    uint32_t file_len = sb.st_size;

    uint32_t arc_cnt = 0;

    uint32_t slice_size = (file_len / (BLOCK_SIZE) + 1) / 4 * (BLOCK_SIZE);

    pthread_t readthread[4];
    Read_Args read_args[4];

    read_args[0].thread_id = 0;
    read_args[0].mmap_start = 0;
    read_args[0].mmap_len = slice_size;
    read_args[0].edge_cnt = 0;
    pthread_create(&readthread[0], NULL, filereader, &read_args[0]);

    read_args[1].thread_id = 1;
    read_args[1].mmap_start = slice_size;
    read_args[1].mmap_len = slice_size;
    read_args[1].edge_cnt = 0;
    pthread_create(&readthread[1], NULL, filereader_merge, &read_args[1]);

    read_args[2].thread_id = 2;
    read_args[2].mmap_start = slice_size * 2;
    read_args[2].mmap_len = slice_size;
    read_args[2].edge_cnt = 0;
    pthread_create(&readthread[2], NULL, filereader, &read_args[2]);

    read_args[3].thread_id = 3;
    read_args[3].mmap_start = slice_size * 3;
    read_args[3].mmap_len = (file_len / (BLOCK_SIZE) + 1) * (BLOCK_SIZE) - slice_size * 3;
    read_args[3].edge_cnt = 0;
    filereader_merge(&read_args[3]);

    pthread_join(readthread[1], NULL);

#ifdef TIMING
    timeval t3, t4;
    gettimeofday(&t3, CLOCK_REALTIME);
#endif

    uint32_t *dst = (uint32_t *) edges;
    uint32_t *src0 = (uint32_t *) edges_t[0];
    uint32_t *src1 = (uint32_t *) edges_t[1];
    uint32_t *end0 = (uint32_t *) (edges_t[0] + read_args[0].edge_cnt + read_args[1].edge_cnt);
    uint32_t *end1 = (uint32_t *) (edges_t[1] + read_args[2].edge_cnt + read_args[3].edge_cnt);
    while (src0 < end0 && src1 < end1) {
        if (*(uint64_t *) src0 < *(uint64_t *) src1) {
            dst[0] = src0[0];
            dst[1] = src0[1];
            dst[2] = src0[2];
            dst[3] = src0[3];
            src0 += 4;
        } else {
            dst[0] = src1[0];
            dst[1] = src1[1];
            dst[2] = src1[2];
            dst[3] = src1[3];
            src1 += 4;
        }
        dst += 4;
    }
    while (src0 < end0) {
        dst[0] = src0[0];
        dst[1] = src0[1];
        dst[2] = src0[2];
        dst[3] = src0[3];
        src0 += 4;
        dst += 4;
    }
    while (src1 < end1) {
        dst[0] = src1[0];
        dst[1] = src1[1];
        dst[2] = src1[2];
        dst[3] = src1[3];
        src1 += 4;
        dst += 4;
    }
#ifdef TIMING
    gettimeofday(&t4, CLOCK_REALTIME);
    printf("last merge: %d\n", calculate_time(t3, t4));
#endif

    close(fd);

    for (int i = 0; i < 4; ++i) {
        arc_cnt += read_args[i].edge_cnt;
    }
    edges[arc_cnt][1] = UINT32_MAX;
#ifdef TIMING
    gettimeofday(&t1, CLOCK_REALTIME);
    printf("read and sort total: %d us\n\n", calculate_time(t2, t1));
#endif

    uint32_t pre = edges[0][1];
    uint32_t cnt = 0;
    id_hash.insert(pre, cnt);
    edges[0][1] = cnt;
    maphead[cnt].ori_id = pre;
    for (uint32_t i = 1; i < arc_cnt; ++i) {
        if (edges[i][1] != pre) {
            ++cnt;
            pre = edges[i][1];
            id_hash.insert(pre, cnt);
            maphead[cnt].ori_id = pre;
            maphead[cnt].head = i;
            maphead[cnt].tail = i;
            edges[i][1] = cnt;
        } else {
            edges[i][1] = cnt;
        }
    }
    ++cnt;
#ifdef TIMING
    gettimeofday(&t2, CLOCK_REALTIME);
    printf("mapid total: %d us\n\n", calculate_time(t1, t2));
#endif
    /* construct dsts(dst+weight),
     * caculate child_num,
     * caculte father_num
     * */
    uint32_t divid[5];
    divid[0] = 0;
    divid[1] = arc_cnt / 4;
    divid[2] = arc_cnt / 4 * 2;
    divid[3] = arc_cnt / 4 * 3;
    divid[4] = arc_cnt;
    pthread_create(&readthread[1], NULL, construct_thread, &divid[1]);
    pthread_create(&readthread[2], NULL, construct_thread, &divid[2]);
    pthread_create(&readthread[3], NULL, construct_thread, &divid[3]);
    construct_thread(divid);
    for (int i = 1; i < 4; i++)
        pthread_join(readthread[i], NULL);

    // for (uint32_t i = 0; i < arc_cnt; ++i) {
    //     if (edges[i][2] == 0)
    //             continue;
    //     auto ite = id_hash.find(edges[i][0]);
    //     if (ite != id_hash.end()) {
    //         uint32_t v = ite->second;
    //         uint32_t offset = maphead[edges[i][1]].tail++;
    //         tempN[offset].id = v;
    //         tempN[offset].weight = edges[i][2];
    //         ++maphead[v].indegree;
    //     }
    // }
#ifdef TIMING
    gettimeofday(&t1, CLOCK_REALTIME);
    printf("construct total: %d us\n\n", calculate_time(t2, t1));
#endif
    /********** topological sort **********/
    uint32_t *const queue = (uint32_t *) edges;  // reuse edges buf as queue
    uint32_t qhead = 0, qtail = 0;
    for (uint32_t i = 0; i < cnt; i++) {
        if (maphead[i].indegree == 0)
            queue[qtail++] = i;
    }
    while (qhead < qtail) {
        uint32_t del = queue[qhead];
        for (uint32_t i = maphead[del].head; i < maphead[del].tail; i++) {
            uint32_t child = tempN[i].id;
            --maphead[child].indegree;
            if (maphead[child].indegree == 0)
                queue[qtail++] = child;
        }
        maphead[del].head = UINT32_MAX;  // delete node
        qhead++;
    }

    /********** constrcut new graph **********/
    uint32_t new_id = 0;
    uint32_t sum_child = 0;
    uint32_t sum_father = 0;
    uint32_t *const graph_tail = (uint32_t *) edges_t[0];
    uint32_t *const fathers_tail = (uint32_t *) edges_t[1];
    for (uint32_t i = 0; i < cnt; i++) {
        if (maphead[i].head != UINT32_MAX) {
            //allocate new id
            maphead[i].new_id = new_id;
            graph[new_id].head = sum_child;
            graph_tail[new_id] = sum_child;
            fathers[new_id].head = sum_father;
            fathers_tail[new_id] = sum_father;
            sum_child += maphead[i].tail - maphead[i].head;
            sum_father += maphead[i].indegree;
            itos_fast_general(id_table[new_id], maphead[i].ori_id);
            ++new_id;
        }
    }
    for (uint32_t i = 0; i < cnt; i++) {
        if (maphead[i].head != UINT32_MAX) {
            for (uint32_t j = maphead[i].head; j < maphead[i].tail; j++) {
                uint32_t new_src_id = maphead[i].new_id;
                uint32_t new_dst_id = maphead[tempN[j].id].new_id;
                uint32_t dst_offset = graph_tail[new_src_id]++;
                uint32_t src_offset = fathers_tail[new_dst_id]++;
                dsts[dst_offset].id = new_dst_id;
                dsts[dst_offset].weight = tempN[j].weight;
                srcs[src_offset].id = new_src_id;
                srcs[src_offset].weight = tempN[j].weight;
            }
        }
    }
    //handle last tail
    graph[new_id].head = graph_tail[new_id - 1];
    fathers[new_id].head = fathers_tail[new_id - 1];
    node_cnt = new_id;
#ifdef TIMING
    gettimeofday(&t2, CLOCK_REALTIME);
    printf("other: %d us\n\n", calculate_time(t1, t2));
#endif
}
void itos_fast_general(uint8_t *const buf, uint32_t id) {
    uint32_t offset = 11;
    uint32_t low = id;
    for (; id >= 1000; id /= 1000) {  // id>999
        low = id % 1000;
        for (int ct = 3; ct > 0; --ct) {
            buf[--offset] = itos_lut[low][ct];
        }
    }
    for (int ct = 3; ct > itos_lut[id][0]; --ct) {
        buf[--offset] = itos_lut[id][ct];
    }
    buf[0] = offset;
}

const char itos_lut[1000][4] = {
        2, '0', '0', '0', 2, '0', '0', '1', 2, '0', '0', '2', 2, '0', '0', '3', 2, '0', '0', '4', 2, '0', '0', '5', 2,
        '0', '0', '6', 2, '0', '0', '7',
        2, '0', '0', '8', 2, '0', '0', '9', 1, '0', '1', '0', 1, '0', '1', '1', 1, '0', '1', '2', 1, '0', '1', '3', 1,
        '0', '1', '4', 1, '0', '1', '5',
        1, '0', '1', '6', 1, '0', '1', '7', 1, '0', '1', '8', 1, '0', '1', '9', 1, '0', '2', '0', 1, '0', '2', '1', 1,
        '0', '2', '2', 1, '0', '2', '3',
        1, '0', '2', '4', 1, '0', '2', '5', 1, '0', '2', '6', 1, '0', '2', '7', 1, '0', '2', '8', 1, '0', '2', '9', 1,
        '0', '3', '0', 1, '0', '3', '1',
        1, '0', '3', '2', 1, '0', '3', '3', 1, '0', '3', '4', 1, '0', '3', '5', 1, '0', '3', '6', 1, '0', '3', '7', 1,
        '0', '3', '8', 1, '0', '3', '9',
        1, '0', '4', '0', 1, '0', '4', '1', 1, '0', '4', '2', 1, '0', '4', '3', 1, '0', '4', '4', 1, '0', '4', '5', 1,
        '0', '4', '6', 1, '0', '4', '7',
        1, '0', '4', '8', 1, '0', '4', '9', 1, '0', '5', '0', 1, '0', '5', '1', 1, '0', '5', '2', 1, '0', '5', '3', 1,
        '0', '5', '4', 1, '0', '5', '5',
        1, '0', '5', '6', 1, '0', '5', '7', 1, '0', '5', '8', 1, '0', '5', '9', 1, '0', '6', '0', 1, '0', '6', '1', 1,
        '0', '6', '2', 1, '0', '6', '3',
        1, '0', '6', '4', 1, '0', '6', '5', 1, '0', '6', '6', 1, '0', '6', '7', 1, '0', '6', '8', 1, '0', '6', '9', 1,
        '0', '7', '0', 1, '0', '7', '1',
        1, '0', '7', '2', 1, '0', '7', '3', 1, '0', '7', '4', 1, '0', '7', '5', 1, '0', '7', '6', 1, '0', '7', '7', 1,
        '0', '7', '8', 1, '0', '7', '9',
        1, '0', '8', '0', 1, '0', '8', '1', 1, '0', '8', '2', 1, '0', '8', '3', 1, '0', '8', '4', 1, '0', '8', '5', 1,
        '0', '8', '6', 1, '0', '8', '7',
        1, '0', '8', '8', 1, '0', '8', '9', 1, '0', '9', '0', 1, '0', '9', '1', 1, '0', '9', '2', 1, '0', '9', '3', 1,
        '0', '9', '4', 1, '0', '9', '5',
        1, '0', '9', '6', 1, '0', '9', '7', 1, '0', '9', '8', 1, '0', '9', '9', 0, '1', '0', '0', 0, '1', '0', '1', 0,
        '1', '0', '2', 0, '1', '0', '3',
        0, '1', '0', '4', 0, '1', '0', '5', 0, '1', '0', '6', 0, '1', '0', '7', 0, '1', '0', '8', 0, '1', '0', '9', 0,
        '1', '1', '0', 0, '1', '1', '1',
        0, '1', '1', '2', 0, '1', '1', '3', 0, '1', '1', '4', 0, '1', '1', '5', 0, '1', '1', '6', 0, '1', '1', '7', 0,
        '1', '1', '8', 0, '1', '1', '9',
        0, '1', '2', '0', 0, '1', '2', '1', 0, '1', '2', '2', 0, '1', '2', '3', 0, '1', '2', '4', 0, '1', '2', '5', 0,
        '1', '2', '6', 0, '1', '2', '7',
        0, '1', '2', '8', 0, '1', '2', '9', 0, '1', '3', '0', 0, '1', '3', '1', 0, '1', '3', '2', 0, '1', '3', '3', 0,
        '1', '3', '4', 0, '1', '3', '5',
        0, '1', '3', '6', 0, '1', '3', '7', 0, '1', '3', '8', 0, '1', '3', '9', 0, '1', '4', '0', 0, '1', '4', '1', 0,
        '1', '4', '2', 0, '1', '4', '3',
        0, '1', '4', '4', 0, '1', '4', '5', 0, '1', '4', '6', 0, '1', '4', '7', 0, '1', '4', '8', 0, '1', '4', '9', 0,
        '1', '5', '0', 0, '1', '5', '1',
        0, '1', '5', '2', 0, '1', '5', '3', 0, '1', '5', '4', 0, '1', '5', '5', 0, '1', '5', '6', 0, '1', '5', '7', 0,
        '1', '5', '8', 0, '1', '5', '9',
        0, '1', '6', '0', 0, '1', '6', '1', 0, '1', '6', '2', 0, '1', '6', '3', 0, '1', '6', '4', 0, '1', '6', '5', 0,
        '1', '6', '6', 0, '1', '6', '7',
        0, '1', '6', '8', 0, '1', '6', '9', 0, '1', '7', '0', 0, '1', '7', '1', 0, '1', '7', '2', 0, '1', '7', '3', 0,
        '1', '7', '4', 0, '1', '7', '5',
        0, '1', '7', '6', 0, '1', '7', '7', 0, '1', '7', '8', 0, '1', '7', '9', 0, '1', '8', '0', 0, '1', '8', '1', 0,
        '1', '8', '2', 0, '1', '8', '3',
        0, '1', '8', '4', 0, '1', '8', '5', 0, '1', '8', '6', 0, '1', '8', '7', 0, '1', '8', '8', 0, '1', '8', '9', 0,
        '1', '9', '0', 0, '1', '9', '1',
        0, '1', '9', '2', 0, '1', '9', '3', 0, '1', '9', '4', 0, '1', '9', '5', 0, '1', '9', '6', 0, '1', '9', '7', 0,
        '1', '9', '8', 0, '1', '9', '9',
        0, '2', '0', '0', 0, '2', '0', '1', 0, '2', '0', '2', 0, '2', '0', '3', 0, '2', '0', '4', 0, '2', '0', '5', 0,
        '2', '0', '6', 0, '2', '0', '7',
        0, '2', '0', '8', 0, '2', '0', '9', 0, '2', '1', '0', 0, '2', '1', '1', 0, '2', '1', '2', 0, '2', '1', '3', 0,
        '2', '1', '4', 0, '2', '1', '5',
        0, '2', '1', '6', 0, '2', '1', '7', 0, '2', '1', '8', 0, '2', '1', '9', 0, '2', '2', '0', 0, '2', '2', '1', 0,
        '2', '2', '2', 0, '2', '2', '3',
        0, '2', '2', '4', 0, '2', '2', '5', 0, '2', '2', '6', 0, '2', '2', '7', 0, '2', '2', '8', 0, '2', '2', '9', 0,
        '2', '3', '0', 0, '2', '3', '1',
        0, '2', '3', '2', 0, '2', '3', '3', 0, '2', '3', '4', 0, '2', '3', '5', 0, '2', '3', '6', 0, '2', '3', '7', 0,
        '2', '3', '8', 0, '2', '3', '9',
        0, '2', '4', '0', 0, '2', '4', '1', 0, '2', '4', '2', 0, '2', '4', '3', 0, '2', '4', '4', 0, '2', '4', '5', 0,
        '2', '4', '6', 0, '2', '4', '7',
        0, '2', '4', '8', 0, '2', '4', '9', 0, '2', '5', '0', 0, '2', '5', '1', 0, '2', '5', '2', 0, '2', '5', '3', 0,
        '2', '5', '4', 0, '2', '5', '5',
        0, '2', '5', '6', 0, '2', '5', '7', 0, '2', '5', '8', 0, '2', '5', '9', 0, '2', '6', '0', 0, '2', '6', '1', 0,
        '2', '6', '2', 0, '2', '6', '3',
        0, '2', '6', '4', 0, '2', '6', '5', 0, '2', '6', '6', 0, '2', '6', '7', 0, '2', '6', '8', 0, '2', '6', '9', 0,
        '2', '7', '0', 0, '2', '7', '1',
        0, '2', '7', '2', 0, '2', '7', '3', 0, '2', '7', '4', 0, '2', '7', '5', 0, '2', '7', '6', 0, '2', '7', '7', 0,
        '2', '7', '8', 0, '2', '7', '9',
        0, '2', '8', '0', 0, '2', '8', '1', 0, '2', '8', '2', 0, '2', '8', '3', 0, '2', '8', '4', 0, '2', '8', '5', 0,
        '2', '8', '6', 0, '2', '8', '7',
        0, '2', '8', '8', 0, '2', '8', '9', 0, '2', '9', '0', 0, '2', '9', '1', 0, '2', '9', '2', 0, '2', '9', '3', 0,
        '2', '9', '4', 0, '2', '9', '5',
        0, '2', '9', '6', 0, '2', '9', '7', 0, '2', '9', '8', 0, '2', '9', '9', 0, '3', '0', '0', 0, '3', '0', '1', 0,
        '3', '0', '2', 0, '3', '0', '3',
        0, '3', '0', '4', 0, '3', '0', '5', 0, '3', '0', '6', 0, '3', '0', '7', 0, '3', '0', '8', 0, '3', '0', '9', 0,
        '3', '1', '0', 0, '3', '1', '1',
        0, '3', '1', '2', 0, '3', '1', '3', 0, '3', '1', '4', 0, '3', '1', '5', 0, '3', '1', '6', 0, '3', '1', '7', 0,
        '3', '1', '8', 0, '3', '1', '9',
        0, '3', '2', '0', 0, '3', '2', '1', 0, '3', '2', '2', 0, '3', '2', '3', 0, '3', '2', '4', 0, '3', '2', '5', 0,
        '3', '2', '6', 0, '3', '2', '7',
        0, '3', '2', '8', 0, '3', '2', '9', 0, '3', '3', '0', 0, '3', '3', '1', 0, '3', '3', '2', 0, '3', '3', '3', 0,
        '3', '3', '4', 0, '3', '3', '5',
        0, '3', '3', '6', 0, '3', '3', '7', 0, '3', '3', '8', 0, '3', '3', '9', 0, '3', '4', '0', 0, '3', '4', '1', 0,
        '3', '4', '2', 0, '3', '4', '3',
        0, '3', '4', '4', 0, '3', '4', '5', 0, '3', '4', '6', 0, '3', '4', '7', 0, '3', '4', '8', 0, '3', '4', '9', 0,
        '3', '5', '0', 0, '3', '5', '1',
        0, '3', '5', '2', 0, '3', '5', '3', 0, '3', '5', '4', 0, '3', '5', '5', 0, '3', '5', '6', 0, '3', '5', '7', 0,
        '3', '5', '8', 0, '3', '5', '9',
        0, '3', '6', '0', 0, '3', '6', '1', 0, '3', '6', '2', 0, '3', '6', '3', 0, '3', '6', '4', 0, '3', '6', '5', 0,
        '3', '6', '6', 0, '3', '6', '7',
        0, '3', '6', '8', 0, '3', '6', '9', 0, '3', '7', '0', 0, '3', '7', '1', 0, '3', '7', '2', 0, '3', '7', '3', 0,
        '3', '7', '4', 0, '3', '7', '5',
        0, '3', '7', '6', 0, '3', '7', '7', 0, '3', '7', '8', 0, '3', '7', '9', 0, '3', '8', '0', 0, '3', '8', '1', 0,
        '3', '8', '2', 0, '3', '8', '3',
        0, '3', '8', '4', 0, '3', '8', '5', 0, '3', '8', '6', 0, '3', '8', '7', 0, '3', '8', '8', 0, '3', '8', '9', 0,
        '3', '9', '0', 0, '3', '9', '1',
        0, '3', '9', '2', 0, '3', '9', '3', 0, '3', '9', '4', 0, '3', '9', '5', 0, '3', '9', '6', 0, '3', '9', '7', 0,
        '3', '9', '8', 0, '3', '9', '9',
        0, '4', '0', '0', 0, '4', '0', '1', 0, '4', '0', '2', 0, '4', '0', '3', 0, '4', '0', '4', 0, '4', '0', '5', 0,
        '4', '0', '6', 0, '4', '0', '7',
        0, '4', '0', '8', 0, '4', '0', '9', 0, '4', '1', '0', 0, '4', '1', '1', 0, '4', '1', '2', 0, '4', '1', '3', 0,
        '4', '1', '4', 0, '4', '1', '5',
        0, '4', '1', '6', 0, '4', '1', '7', 0, '4', '1', '8', 0, '4', '1', '9', 0, '4', '2', '0', 0, '4', '2', '1', 0,
        '4', '2', '2', 0, '4', '2', '3',
        0, '4', '2', '4', 0, '4', '2', '5', 0, '4', '2', '6', 0, '4', '2', '7', 0, '4', '2', '8', 0, '4', '2', '9', 0,
        '4', '3', '0', 0, '4', '3', '1',
        0, '4', '3', '2', 0, '4', '3', '3', 0, '4', '3', '4', 0, '4', '3', '5', 0, '4', '3', '6', 0, '4', '3', '7', 0,
        '4', '3', '8', 0, '4', '3', '9',
        0, '4', '4', '0', 0, '4', '4', '1', 0, '4', '4', '2', 0, '4', '4', '3', 0, '4', '4', '4', 0, '4', '4', '5', 0,
        '4', '4', '6', 0, '4', '4', '7',
        0, '4', '4', '8', 0, '4', '4', '9', 0, '4', '5', '0', 0, '4', '5', '1', 0, '4', '5', '2', 0, '4', '5', '3', 0,
        '4', '5', '4', 0, '4', '5', '5',
        0, '4', '5', '6', 0, '4', '5', '7', 0, '4', '5', '8', 0, '4', '5', '9', 0, '4', '6', '0', 0, '4', '6', '1', 0,
        '4', '6', '2', 0, '4', '6', '3',
        0, '4', '6', '4', 0, '4', '6', '5', 0, '4', '6', '6', 0, '4', '6', '7', 0, '4', '6', '8', 0, '4', '6', '9', 0,
        '4', '7', '0', 0, '4', '7', '1',
        0, '4', '7', '2', 0, '4', '7', '3', 0, '4', '7', '4', 0, '4', '7', '5', 0, '4', '7', '6', 0, '4', '7', '7', 0,
        '4', '7', '8', 0, '4', '7', '9',
        0, '4', '8', '0', 0, '4', '8', '1', 0, '4', '8', '2', 0, '4', '8', '3', 0, '4', '8', '4', 0, '4', '8', '5', 0,
        '4', '8', '6', 0, '4', '8', '7',
        0, '4', '8', '8', 0, '4', '8', '9', 0, '4', '9', '0', 0, '4', '9', '1', 0, '4', '9', '2', 0, '4', '9', '3', 0,
        '4', '9', '4', 0, '4', '9', '5',
        0, '4', '9', '6', 0, '4', '9', '7', 0, '4', '9', '8', 0, '4', '9', '9', 0, '5', '0', '0', 0, '5', '0', '1', 0,
        '5', '0', '2', 0, '5', '0', '3',
        0, '5', '0', '4', 0, '5', '0', '5', 0, '5', '0', '6', 0, '5', '0', '7', 0, '5', '0', '8', 0, '5', '0', '9', 0,
        '5', '1', '0', 0, '5', '1', '1',
        0, '5', '1', '2', 0, '5', '1', '3', 0, '5', '1', '4', 0, '5', '1', '5', 0, '5', '1', '6', 0, '5', '1', '7', 0,
        '5', '1', '8', 0, '5', '1', '9',
        0, '5', '2', '0', 0, '5', '2', '1', 0, '5', '2', '2', 0, '5', '2', '3', 0, '5', '2', '4', 0, '5', '2', '5', 0,
        '5', '2', '6', 0, '5', '2', '7',
        0, '5', '2', '8', 0, '5', '2', '9', 0, '5', '3', '0', 0, '5', '3', '1', 0, '5', '3', '2', 0, '5', '3', '3', 0,
        '5', '3', '4', 0, '5', '3', '5',
        0, '5', '3', '6', 0, '5', '3', '7', 0, '5', '3', '8', 0, '5', '3', '9', 0, '5', '4', '0', 0, '5', '4', '1', 0,
        '5', '4', '2', 0, '5', '4', '3',
        0, '5', '4', '4', 0, '5', '4', '5', 0, '5', '4', '6', 0, '5', '4', '7', 0, '5', '4', '8', 0, '5', '4', '9', 0,
        '5', '5', '0', 0, '5', '5', '1',
        0, '5', '5', '2', 0, '5', '5', '3', 0, '5', '5', '4', 0, '5', '5', '5', 0, '5', '5', '6', 0, '5', '5', '7', 0,
        '5', '5', '8', 0, '5', '5', '9',
        0, '5', '6', '0', 0, '5', '6', '1', 0, '5', '6', '2', 0, '5', '6', '3', 0, '5', '6', '4', 0, '5', '6', '5', 0,
        '5', '6', '6', 0, '5', '6', '7',
        0, '5', '6', '8', 0, '5', '6', '9', 0, '5', '7', '0', 0, '5', '7', '1', 0, '5', '7', '2', 0, '5', '7', '3', 0,
        '5', '7', '4', 0, '5', '7', '5',
        0, '5', '7', '6', 0, '5', '7', '7', 0, '5', '7', '8', 0, '5', '7', '9', 0, '5', '8', '0', 0, '5', '8', '1', 0,
        '5', '8', '2', 0, '5', '8', '3',
        0, '5', '8', '4', 0, '5', '8', '5', 0, '5', '8', '6', 0, '5', '8', '7', 0, '5', '8', '8', 0, '5', '8', '9', 0,
        '5', '9', '0', 0, '5', '9', '1',
        0, '5', '9', '2', 0, '5', '9', '3', 0, '5', '9', '4', 0, '5', '9', '5', 0, '5', '9', '6', 0, '5', '9', '7', 0,
        '5', '9', '8', 0, '5', '9', '9',
        0, '6', '0', '0', 0, '6', '0', '1', 0, '6', '0', '2', 0, '6', '0', '3', 0, '6', '0', '4', 0, '6', '0', '5', 0,
        '6', '0', '6', 0, '6', '0', '7',
        0, '6', '0', '8', 0, '6', '0', '9', 0, '6', '1', '0', 0, '6', '1', '1', 0, '6', '1', '2', 0, '6', '1', '3', 0,
        '6', '1', '4', 0, '6', '1', '5',
        0, '6', '1', '6', 0, '6', '1', '7', 0, '6', '1', '8', 0, '6', '1', '9', 0, '6', '2', '0', 0, '6', '2', '1', 0,
        '6', '2', '2', 0, '6', '2', '3',
        0, '6', '2', '4', 0, '6', '2', '5', 0, '6', '2', '6', 0, '6', '2', '7', 0, '6', '2', '8', 0, '6', '2', '9', 0,
        '6', '3', '0', 0, '6', '3', '1',
        0, '6', '3', '2', 0, '6', '3', '3', 0, '6', '3', '4', 0, '6', '3', '5', 0, '6', '3', '6', 0, '6', '3', '7', 0,
        '6', '3', '8', 0, '6', '3', '9',
        0, '6', '4', '0', 0, '6', '4', '1', 0, '6', '4', '2', 0, '6', '4', '3', 0, '6', '4', '4', 0, '6', '4', '5', 0,
        '6', '4', '6', 0, '6', '4', '7',
        0, '6', '4', '8', 0, '6', '4', '9', 0, '6', '5', '0', 0, '6', '5', '1', 0, '6', '5', '2', 0, '6', '5', '3', 0,
        '6', '5', '4', 0, '6', '5', '5',
        0, '6', '5', '6', 0, '6', '5', '7', 0, '6', '5', '8', 0, '6', '5', '9', 0, '6', '6', '0', 0, '6', '6', '1', 0,
        '6', '6', '2', 0, '6', '6', '3',
        0, '6', '6', '4', 0, '6', '6', '5', 0, '6', '6', '6', 0, '6', '6', '7', 0, '6', '6', '8', 0, '6', '6', '9', 0,
        '6', '7', '0', 0, '6', '7', '1',
        0, '6', '7', '2', 0, '6', '7', '3', 0, '6', '7', '4', 0, '6', '7', '5', 0, '6', '7', '6', 0, '6', '7', '7', 0,
        '6', '7', '8', 0, '6', '7', '9',
        0, '6', '8', '0', 0, '6', '8', '1', 0, '6', '8', '2', 0, '6', '8', '3', 0, '6', '8', '4', 0, '6', '8', '5', 0,
        '6', '8', '6', 0, '6', '8', '7',
        0, '6', '8', '8', 0, '6', '8', '9', 0, '6', '9', '0', 0, '6', '9', '1', 0, '6', '9', '2', 0, '6', '9', '3', 0,
        '6', '9', '4', 0, '6', '9', '5',
        0, '6', '9', '6', 0, '6', '9', '7', 0, '6', '9', '8', 0, '6', '9', '9', 0, '7', '0', '0', 0, '7', '0', '1', 0,
        '7', '0', '2', 0, '7', '0', '3',
        0, '7', '0', '4', 0, '7', '0', '5', 0, '7', '0', '6', 0, '7', '0', '7', 0, '7', '0', '8', 0, '7', '0', '9', 0,
        '7', '1', '0', 0, '7', '1', '1',
        0, '7', '1', '2', 0, '7', '1', '3', 0, '7', '1', '4', 0, '7', '1', '5', 0, '7', '1', '6', 0, '7', '1', '7', 0,
        '7', '1', '8', 0, '7', '1', '9',
        0, '7', '2', '0', 0, '7', '2', '1', 0, '7', '2', '2', 0, '7', '2', '3', 0, '7', '2', '4', 0, '7', '2', '5', 0,
        '7', '2', '6', 0, '7', '2', '7',
        0, '7', '2', '8', 0, '7', '2', '9', 0, '7', '3', '0', 0, '7', '3', '1', 0, '7', '3', '2', 0, '7', '3', '3', 0,
        '7', '3', '4', 0, '7', '3', '5',
        0, '7', '3', '6', 0, '7', '3', '7', 0, '7', '3', '8', 0, '7', '3', '9', 0, '7', '4', '0', 0, '7', '4', '1', 0,
        '7', '4', '2', 0, '7', '4', '3',
        0, '7', '4', '4', 0, '7', '4', '5', 0, '7', '4', '6', 0, '7', '4', '7', 0, '7', '4', '8', 0, '7', '4', '9', 0,
        '7', '5', '0', 0, '7', '5', '1',
        0, '7', '5', '2', 0, '7', '5', '3', 0, '7', '5', '4', 0, '7', '5', '5', 0, '7', '5', '6', 0, '7', '5', '7', 0,
        '7', '5', '8', 0, '7', '5', '9',
        0, '7', '6', '0', 0, '7', '6', '1', 0, '7', '6', '2', 0, '7', '6', '3', 0, '7', '6', '4', 0, '7', '6', '5', 0,
        '7', '6', '6', 0, '7', '6', '7',
        0, '7', '6', '8', 0, '7', '6', '9', 0, '7', '7', '0', 0, '7', '7', '1', 0, '7', '7', '2', 0, '7', '7', '3', 0,
        '7', '7', '4', 0, '7', '7', '5',
        0, '7', '7', '6', 0, '7', '7', '7', 0, '7', '7', '8', 0, '7', '7', '9', 0, '7', '8', '0', 0, '7', '8', '1', 0,
        '7', '8', '2', 0, '7', '8', '3',
        0, '7', '8', '4', 0, '7', '8', '5', 0, '7', '8', '6', 0, '7', '8', '7', 0, '7', '8', '8', 0, '7', '8', '9', 0,
        '7', '9', '0', 0, '7', '9', '1',
        0, '7', '9', '2', 0, '7', '9', '3', 0, '7', '9', '4', 0, '7', '9', '5', 0, '7', '9', '6', 0, '7', '9', '7', 0,
        '7', '9', '8', 0, '7', '9', '9',
        0, '8', '0', '0', 0, '8', '0', '1', 0, '8', '0', '2', 0, '8', '0', '3', 0, '8', '0', '4', 0, '8', '0', '5', 0,
        '8', '0', '6', 0, '8', '0', '7',
        0, '8', '0', '8', 0, '8', '0', '9', 0, '8', '1', '0', 0, '8', '1', '1', 0, '8', '1', '2', 0, '8', '1', '3', 0,
        '8', '1', '4', 0, '8', '1', '5',
        0, '8', '1', '6', 0, '8', '1', '7', 0, '8', '1', '8', 0, '8', '1', '9', 0, '8', '2', '0', 0, '8', '2', '1', 0,
        '8', '2', '2', 0, '8', '2', '3',
        0, '8', '2', '4', 0, '8', '2', '5', 0, '8', '2', '6', 0, '8', '2', '7', 0, '8', '2', '8', 0, '8', '2', '9', 0,
        '8', '3', '0', 0, '8', '3', '1',
        0, '8', '3', '2', 0, '8', '3', '3', 0, '8', '3', '4', 0, '8', '3', '5', 0, '8', '3', '6', 0, '8', '3', '7', 0,
        '8', '3', '8', 0, '8', '3', '9',
        0, '8', '4', '0', 0, '8', '4', '1', 0, '8', '4', '2', 0, '8', '4', '3', 0, '8', '4', '4', 0, '8', '4', '5', 0,
        '8', '4', '6', 0, '8', '4', '7',
        0, '8', '4', '8', 0, '8', '4', '9', 0, '8', '5', '0', 0, '8', '5', '1', 0, '8', '5', '2', 0, '8', '5', '3', 0,
        '8', '5', '4', 0, '8', '5', '5',
        0, '8', '5', '6', 0, '8', '5', '7', 0, '8', '5', '8', 0, '8', '5', '9', 0, '8', '6', '0', 0, '8', '6', '1', 0,
        '8', '6', '2', 0, '8', '6', '3',
        0, '8', '6', '4', 0, '8', '6', '5', 0, '8', '6', '6', 0, '8', '6', '7', 0, '8', '6', '8', 0, '8', '6', '9', 0,
        '8', '7', '0', 0, '8', '7', '1',
        0, '8', '7', '2', 0, '8', '7', '3', 0, '8', '7', '4', 0, '8', '7', '5', 0, '8', '7', '6', 0, '8', '7', '7', 0,
        '8', '7', '8', 0, '8', '7', '9',
        0, '8', '8', '0', 0, '8', '8', '1', 0, '8', '8', '2', 0, '8', '8', '3', 0, '8', '8', '4', 0, '8', '8', '5', 0,
        '8', '8', '6', 0, '8', '8', '7',
        0, '8', '8', '8', 0, '8', '8', '9', 0, '8', '9', '0', 0, '8', '9', '1', 0, '8', '9', '2', 0, '8', '9', '3', 0,
        '8', '9', '4', 0, '8', '9', '5',
        0, '8', '9', '6', 0, '8', '9', '7', 0, '8', '9', '8', 0, '8', '9', '9', 0, '9', '0', '0', 0, '9', '0', '1', 0,
        '9', '0', '2', 0, '9', '0', '3',
        0, '9', '0', '4', 0, '9', '0', '5', 0, '9', '0', '6', 0, '9', '0', '7', 0, '9', '0', '8', 0, '9', '0', '9', 0,
        '9', '1', '0', 0, '9', '1', '1',
        0, '9', '1', '2', 0, '9', '1', '3', 0, '9', '1', '4', 0, '9', '1', '5', 0, '9', '1', '6', 0, '9', '1', '7', 0,
        '9', '1', '8', 0, '9', '1', '9',
        0, '9', '2', '0', 0, '9', '2', '1', 0, '9', '2', '2', 0, '9', '2', '3', 0, '9', '2', '4', 0, '9', '2', '5', 0,
        '9', '2', '6', 0, '9', '2', '7',
        0, '9', '2', '8', 0, '9', '2', '9', 0, '9', '3', '0', 0, '9', '3', '1', 0, '9', '3', '2', 0, '9', '3', '3', 0,
        '9', '3', '4', 0, '9', '3', '5',
        0, '9', '3', '6', 0, '9', '3', '7', 0, '9', '3', '8', 0, '9', '3', '9', 0, '9', '4', '0', 0, '9', '4', '1', 0,
        '9', '4', '2', 0, '9', '4', '3',
        0, '9', '4', '4', 0, '9', '4', '5', 0, '9', '4', '6', 0, '9', '4', '7', 0, '9', '4', '8', 0, '9', '4', '9', 0,
        '9', '5', '0', 0, '9', '5', '1',
        0, '9', '5', '2', 0, '9', '5', '3', 0, '9', '5', '4', 0, '9', '5', '5', 0, '9', '5', '6', 0, '9', '5', '7', 0,
        '9', '5', '8', 0, '9', '5', '9',
        0, '9', '6', '0', 0, '9', '6', '1', 0, '9', '6', '2', 0, '9', '6', '3', 0, '9', '6', '4', 0, '9', '6', '5', 0,
        '9', '6', '6', 0, '9', '6', '7',
        0, '9', '6', '8', 0, '9', '6', '9', 0, '9', '7', '0', 0, '9', '7', '1', 0, '9', '7', '2', 0, '9', '7', '3', 0,
        '9', '7', '4', 0, '9', '7', '5',
        0, '9', '7', '6', 0, '9', '7', '7', 0, '9', '7', '8', 0, '9', '7', '9', 0, '9', '8', '0', 0, '9', '8', '1', 0,
        '9', '8', '2', 0, '9', '8', '3',
        0, '9', '8', '4', 0, '9', '8', '5', 0, '9', '8', '6', 0, '9', '8', '7', 0, '9', '8', '8', 0, '9', '8', '9', 0,
        '9', '9', '0', 0, '9', '9', '1',
        0, '9', '9', '2', 0, '9', '9', '3', 0, '9', '9', '4', 0, '9', '9', '5', 0, '9', '9', '6', 0, '9', '9', '7', 0,
        '9', '9', '8', 0, '9', '9', '9',
};