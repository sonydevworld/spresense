uint32_t find_primes(uint32_t start_val, uint32_t limit) {
    uint32_t primes = 0;

    for (; start_val <= limit; start_val++) {
	int x = 2;
	while (x <= start_val) {
	    if (start_val % x == 0)
		break;
	    x++;
	}
	if (x == start_val)
	    primes++;
    }
    return primes;
}
