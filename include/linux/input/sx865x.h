#ifndef __SX865X_H__
#define __SX865X_H__

#define SX865X_INVERT_X	(1 << 0)
#define SX865X_INVERT_Y	(1 << 1)
#define SX865X_SWAP_XY	(1 << 2)

struct sx865x_platform_data {
	u16 x_plate_ohms;
	u16 y_plate_ohms;
	u16 flags;
};

#endif /* __SX865X_H__ */
