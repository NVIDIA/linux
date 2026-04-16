struct ast2600_ara {
	struct i2c_client       *ara_i2c_client;
	struct i2c_client       *client;
	spinlock_t              *lock_addr_reg;
};

struct ast2600_ara* register_ast2600_ara(struct i2c_client *client,
					 spinlock_t *lock_addr_reg);
void unregister_ast2600_ara(struct ast2600_ara *ara);
void enable_ast2600_ara(struct i2c_client *client);
