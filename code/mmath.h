#ifndef CODE_MMATH_H_
#define CODE_MMATH_H_

#define BIT(n)                  (1 << n)
#define BIT_SET_TRUE(x, mask)   ((x) |= (mask))
#define BIT_SET_FALSE(x, mask)  ((x) &= ~(mask))
#define BIT_TOGGLE(x, mask)     ((x) ^= (mask))
#define BIT_IS_TRUE(x, mask)    ((x & mask) != 0)
#define BIT_IS_FALSE(x, mask)   ((x & mask) == 0)

#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof(arr[0]))

int mapLinear(int value, int in_min, int in_max, int out_min, int out_max);
int mapTri(int value, int in_min, int in_mid, int in_max, int out_min, int out_mid, int out_max);

#endif /* CODE_MMATH_H_ */
