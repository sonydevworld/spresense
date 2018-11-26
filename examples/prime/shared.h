#pragma once

/* How many worker CPUs to run the same instance from*/
#define N_CPU 5

#define KEY_SHM   1
#define KEY_MQ    2
/* One mutex for all CPUs, probably unnecessary since we are not using shared data between the various worker instances. */
#define KEY_MUTEX 3

#define PRIME_LIMIT 15000



typedef struct {
    uint32_t start;         /* Range start. */
    uint32_t n;             /* Range amount of numbers. */
    uint32_t primes_found;  /* Result: The amount of prime values found in the range [start:start+n] */
} prime_data_t;

/* Struct holding the data that is being shared between the CPU:s. */
typedef struct {
    prime_data_t prime[N_CPU];
} shared_data_t;
