/* I2C Global Registers */
/* 0x00 : I2CG Interrupt Status Register  */
/* 0x08 : I2CG Interrupt Target Assignment  */
/* 0x0c : I2CG Global Control Register (AST2500)  */
#define ASPEED_I2CG_GLOBAL_CTRL_REG 0x0c
#define ASPEED_I2CG_SRAM_BUFFER_EN BIT(0)

/* I2C Register */
#define ASPEED_I2C_FUN_CTRL_REG				0x00
#define ASPEED_I2C_AC_TIMING_REG1			0x04
#define ASPEED_I2C_AC_TIMING_REG2			0x08
#define ASPEED_I2C_INTR_CTRL_REG			0x0c
#define ASPEED_I2C_INTR_STS_REG				0x10
#define ASPEED_I2C_CMD_REG				0x14
#define ASPEED_I2C_DEV_ADDR_REG				0x18
#define ASPEED_I2C_BYTE_BUF_REG				0x20

/* Global Register Definition */
/* 0x00 : I2C Interrupt Status Register  */
/* 0x08 : I2C Interrupt Target Assignment  */

/* Device Register Definition */
/* 0x00 : I2CD Function Control Register  */
#define ASPEED_I2CD_MULTI_MASTER_DIS			BIT(15)
#define ASPEED_I2CD_SDA_DRIVE_1T_EN			BIT(8)
#define ASPEED_I2CD_M_SDA_DRIVE_1T_EN			BIT(7)
#define ASPEED_I2CD_M_HIGH_SPEED_EN			BIT(6)
#define ASPEED_I2CD_SLAVE_EN				BIT(1)
#define ASPEED_I2CD_MASTER_EN				BIT(0)

/* 0x04 : I2CD Clock and AC Timing Control Register #1 */
#define ASPEED_I2CD_TIME_TBUF_MASK			GENMASK(31, 28)
#define ASPEED_I2CD_TIME_THDSTA_MASK			GENMASK(27, 24)
#define ASPEED_I2CD_TIME_TACST_MASK			GENMASK(23, 20)
#define ASPEED_I2CD_TIME_SCL_HIGH_SHIFT			16
#define ASPEED_I2CD_TIME_SCL_HIGH_MASK			GENMASK(19, 16)
#define ASPEED_I2CD_TIME_SCL_LOW_SHIFT			12
#define ASPEED_I2CD_TIME_SCL_LOW_MASK			GENMASK(15, 12)
#define ASPEED_I2CD_TIME_BASE_DIVISOR_MASK		GENMASK(3, 0)
#define ASPEED_I2CD_TIME_SCL_REG_MAX			GENMASK(3, 0)
/* 0x08 : I2CD Clock and AC Timing Control Register #2 */
#define ASPEED_NO_TIMEOUT_CTRL				0

/* 0x0c : I2CD Interrupt Control Register &
 * 0x10 : I2CD Interrupt Status Register
 *
 * These share bit definitions, so use the same values for the enable &
 * status bits.
 */
#define ASPEED_2500_I2CD_SLAVE_ADDR_MATCH_INDICATOR_MASK 0x1
#define ASPEED_2500_I2CD_SLAVE_ADDR_MATCH_INDICATOR_OFFESET 31
#define ASPEED_2600_I2CD_SLAVE_ADDR_MATCH_INDICATOR_MASK 0x3
#define ASPEED_2600_I2CD_SLAVE_ADDR_MATCH_INDICATOR_OFFESET 30

#define ASPEED_I2CD_INTR_RECV_MASK			0xf000efff
#define ASPEED_I2CD_INTR_SDA_DL_TIMEOUT			BIT(14)
#define ASPEED_I2CD_INTR_BUS_RECOVER_DONE		BIT(13)
#define ASPEED_I2CD_INTR_SLAVE_MATCH			BIT(7)
#define ASPEED_I2CD_INTR_SCL_TIMEOUT			BIT(6)
#define ASPEED_I2CD_INTR_ABNORMAL			BIT(5)
#define ASPEED_I2CD_INTR_NORMAL_STOP			BIT(4)
#define ASPEED_I2CD_INTR_ARBIT_LOSS			BIT(3)
#define ASPEED_I2CD_INTR_RX_DONE			BIT(2)
#define ASPEED_I2CD_INTR_TX_NAK				BIT(1)
#define ASPEED_I2CD_INTR_TX_ACK				BIT(0)
#define ASPEED_I2CD_INTR_MASTER_ERRORS					       \
		(ASPEED_I2CD_INTR_SDA_DL_TIMEOUT |			       \
		 ASPEED_I2CD_INTR_SCL_TIMEOUT |				       \
		 ASPEED_I2CD_INTR_ABNORMAL |				       \
		 ASPEED_I2CD_INTR_ARBIT_LOSS)
#define ASPEED_I2CD_INTR_ALL						       \
		(ASPEED_I2CD_INTR_SDA_DL_TIMEOUT |			       \
		 ASPEED_I2CD_INTR_BUS_RECOVER_DONE |			       \
		 ASPEED_I2CD_INTR_SCL_TIMEOUT |				       \
		 ASPEED_I2CD_INTR_ABNORMAL |				       \
		 ASPEED_I2CD_INTR_NORMAL_STOP |				       \
		 ASPEED_I2CD_INTR_ARBIT_LOSS |				       \
		 ASPEED_I2CD_INTR_RX_DONE |				       \
		 ASPEED_I2CD_INTR_TX_NAK |				       \
		 ASPEED_I2CD_INTR_TX_ACK)

/* 0x14 : I2CD Command/Status Register   */
#define ASPEED_I2CD_SCL_LINE_STS			BIT(18)
#define ASPEED_I2CD_SDA_LINE_STS			BIT(17)
#define ASPEED_I2CD_BUS_BUSY_STS			BIT(16)
#define ASPEED_I2CD_BUS_RECOVER_CMD			BIT(11)

/* Command Bit */
#define ASPEED_I2CD_M_STOP_CMD				BIT(5)
#define ASPEED_I2CD_M_S_RX_CMD_LAST			BIT(4)
#define ASPEED_I2CD_M_RX_CMD				BIT(3)
#define ASPEED_I2CD_S_TX_CMD				BIT(2)
#define ASPEED_I2CD_M_TX_CMD				BIT(1)
#define ASPEED_I2CD_M_START_CMD				BIT(0)
#define ASPEED_I2CD_MASTER_CMDS_MASK					       \
		(ASPEED_I2CD_M_STOP_CMD |				       \
		 ASPEED_I2CD_M_S_RX_CMD_LAST |				       \
		 ASPEED_I2CD_M_RX_CMD |					       \
		 ASPEED_I2CD_M_TX_CMD |					       \
		 ASPEED_I2CD_M_START_CMD)

/* 0x18 : I2CD Slave Device Address Register   */
#define ASPEED_I2CD_EN_SLAVE_DEV_ADDR3			BIT(23)
#define ASPEED_I2CD_DIS_SLAVE_DEV_ADDR3			~ASPEED_I2CD_EN_SLAVE_DEV_ADDR3
#define ASPEED_I2CD_DEV_ADDR3_MASK			GENMASK(22, 16)
#define ASPEED_I2CD_DEV_ADDR3_SHIFT			16
#define ASPEED_I2CD_EN_SLAVE_DEV_ADDR2			BIT(15)
#define ASPEED_I2CD_DIS_SLAVE_DEV_ADDR2			~ASPEED_I2CD_EN_SLAVE_DEV_ADDR2
#define ASPEED_I2CD_DEV_ADDR2_MASK			GENMASK(14, 8)
#define ASPEED_I2CD_DEV_ADDR2_SHIFT			8
#define ASPEED_I2CD_DEV_ADDR1_MASK			GENMASK(6, 0)
#define ASPEED_I2CD_DEV_ADDR1_SHIFT			0

enum aspeed_i2c_master_state {
	ASPEED_I2C_MASTER_INACTIVE,
	ASPEED_I2C_MASTER_START,
	ASPEED_I2C_MASTER_TX_FIRST,
	ASPEED_I2C_MASTER_TX,
	ASPEED_I2C_MASTER_RX_FIRST,
	ASPEED_I2C_MASTER_RX,
	ASPEED_I2C_MASTER_STOP,
};

enum aspeed_i2c_slave_state {
	ASPEED_I2C_SLAVE_INACTIVE,
	ASPEED_I2C_SLAVE_START,
	ASPEED_I2C_SLAVE_READ_REQUESTED,
	ASPEED_I2C_SLAVE_READ_PROCESSED,
	ASPEED_I2C_SLAVE_WRITE_REQUESTED,
	ASPEED_I2C_SLAVE_WRITE_RECEIVED,
	ASPEED_I2C_SLAVE_STOP,
};

#define ASPEED_I2C_MAX_SLAVE 0x3

struct aspeed_i2c_bus {
	struct i2c_adapter		adap;
	struct device			*dev;
	void __iomem			*base;
	struct reset_control		*rst;
	/* Synchronizes I/O mem access to base. */
	spinlock_t			lock;
	struct completion		cmd_complete;
	u32				(*get_clk_reg_val)(struct device *dev,
							   u32 divisor);
	unsigned long			parent_clk_frequency;
	u32				bus_frequency;
	/* Transaction state. */
	enum aspeed_i2c_master_state	master_state;
	struct i2c_msg			*msgs;
	size_t				buf_index;
	size_t				msgs_index;
	size_t				msgs_count;
	bool				send_stop;
	int				cmd_err;
	/* Protected only by i2c_lock_bus */
	int				master_xfer_result;
	/* Multi-master */
	bool				multi_master;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	struct i2c_client		*slave[ASPEED_I2C_MAX_SLAVE];
	enum aspeed_i2c_slave_state	slave_state[ASPEED_I2C_MAX_SLAVE];
	u32 slave_addr_ind_mask;
	u32 slave_addr_ind_offset;
	u32 slave_addr_ind;
	int max_slaves_enable;
#endif /* CONFIG_I2C_SLAVE */
};


