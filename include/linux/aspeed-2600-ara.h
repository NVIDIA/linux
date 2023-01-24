struct ast2600_ara {
	struct i2c_client       *ara_i2c_client;
	struct i2c_client       *client;
};

struct ast2600_ara* register_ast2600_ara(struct i2c_client *client);
void unregister_ast2600_ara(struct ast2600_ara *ara);
void enable_ast2600_ara(struct i2c_client *client);
