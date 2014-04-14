/*
 * Register map access API - SMBus support
 *
 * Copyright 2014 Free Electrons
 *
 * Author: Boris Brezillon <boris.brezillon@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>

struct regmap_smbus_context {
	struct i2c_client *i2c;
	enum regmap_smbus_transfer_type transfer_type;
	int val_bytes;
};

static int regmap_smbus_write(void *context, const void *data, size_t count)
{
	struct regmap_smbus_context *ctx = context;
	int ret = 0;
	u8 reg = *(u8 *)data++;

	count--;

	switch (ctx->transfer_type) {
	case REGMAP_SMBUS_BYTE_TRANSFER:
		while (count > 0 && !ret) {
			ret = i2c_smbus_write_byte_data(ctx->i2c, reg++,
							*(u8 *)data++);

			count--;
		}
		break;

	case REGMAP_SMBUS_WORD_TRANSFER:
		while (count > 0 && !ret) {
			ret = i2c_smbus_write_word_data(ctx->i2c, reg,
							*(u16 *)data++);

			reg += 2;
			count -= 2;
		}
		break;

	case REGMAP_SMBUS_BLOCK_TRANSFER:
		while (count > 0 && !ret) {
			ret = i2c_smbus_write_block_data(ctx->i2c,
							 reg,
							 ctx->val_bytes,
							 (const u8 *)data);

			reg += ctx->val_bytes;
			count -= ctx->val_bytes;
			data += ctx->val_bytes;
		}
		break;

	case REGMAP_SMBUS_I2C_BLOCK_TRANSFER:
		while (count > 0 && !ret) {
			ret = i2c_smbus_write_i2c_block_data(ctx->i2c,
							     reg,
							     ctx->val_bytes,
							     (const u8 *)data);

			reg += ctx->val_bytes;
			count -= ctx->val_bytes;
			data += ctx->val_bytes;
		}
		break;

	default:
		return -ENOTSUPP;
	}

	return ret;
}

static int regmap_smbus_gather_write(void *context,
				     const void *reg, size_t reg_size,
				     const void *val, size_t val_size)
{
	struct regmap_smbus_context *ctx = context;
	u8 smbus_reg;
	int ret = 0;

	if (reg_size != 1)
		return -ENOTSUPP;

	smbus_reg = *(u8 *)reg;

	switch (ctx->transfer_type) {
	case REGMAP_SMBUS_BYTE_TRANSFER:
		while (val_size && !ret) {
			ret = i2c_smbus_write_byte_data(ctx->i2c,
							smbus_reg++,
							*(u8 *)val++);

			val_size--;
		}
		break;

	case REGMAP_SMBUS_WORD_TRANSFER:
		while (val_size && !ret) {
			ret = i2c_smbus_write_word_data(ctx->i2c,
							smbus_reg,
							*(u16 *)val++);

			smbus_reg += 2;
			val_size -= 2;
		}
		break;

	case REGMAP_SMBUS_BLOCK_TRANSFER:
		while (val_size && !ret) {
			ret = i2c_smbus_write_block_data(ctx->i2c,
							 smbus_reg,
							 ctx->val_bytes,
							 (const u8 *)val);

			smbus_reg += ctx->val_bytes;
			val_size -= ctx->val_bytes;
			val += ctx->val_bytes;
		}
		break;

	case REGMAP_SMBUS_I2C_BLOCK_TRANSFER:
		if (val_size > I2C_SMBUS_BLOCK_MAX)
			return -ENOTSUPP;

		while (val_size && !ret) {
			ret = i2c_smbus_write_i2c_block_data(ctx->i2c,
							     smbus_reg,
							     ctx->val_bytes,
							     (const u8 *)val);

			smbus_reg += ctx->val_bytes;
			val_size -= ctx->val_bytes;
			val += ctx->val_bytes;
		}
		break;

	default:
		return -ENOTSUPP;
	}

	return ret;
}

static int regmap_smbus_read(void *context,
			     const void *reg, size_t reg_size,
			     void *val, size_t val_size)
{
	struct regmap_smbus_context *ctx = context;
	u8 buffer[I2C_SMBUS_BLOCK_MAX];
	u8 smbus_reg;
	int ret = 0;

	if (reg_size != 1)
		return -ENOTSUPP;

	smbus_reg = *(u8 *)reg;

	switch (ctx->transfer_type) {
	case REGMAP_SMBUS_BYTE_TRANSFER:
		while (val_size && ret >= 0) {
			ret = i2c_smbus_read_byte_data(ctx->i2c, smbus_reg++);
			if (ret >= 0)
				*((u8 *)val++) = ret;

			val_size--;
		}
		break;

	case REGMAP_SMBUS_WORD_TRANSFER:
		while (val_size && ret >= 0) {
			ret = i2c_smbus_read_word_data(ctx->i2c, smbus_reg);
			if (ret >= 0)
				*(u16 *)val++ = ret;

			smbus_reg += 2;
			val_size -= 2;
		}
		break;

	case REGMAP_SMBUS_BLOCK_TRANSFER:
		while (val_size && ret >= 0) {
			ret = i2c_smbus_read_block_data(ctx->i2c,
							smbus_reg,
							buffer);
			if (ret >= 0 && ret < ctx->val_bytes) {
				ret = -EIO;
				break;
			}

			memcpy(val, buffer, ctx->val_bytes);
			smbus_reg += ctx->val_bytes;
			val_size -= ctx->val_bytes;
			val += ctx->val_bytes;
		}
		break;

	case REGMAP_SMBUS_I2C_BLOCK_TRANSFER:
		while (val_size && ret >= 0) {
			return -ENOTSUPP;

			ret = i2c_smbus_read_i2c_block_data(ctx->i2c,
							    smbus_reg,
							    ctx->val_bytes,
							    (u8 *)val);
			if (ret >= 0 && ret < val_size) {
				ret = -EIO;
				break;
			}

			smbus_reg += ctx->val_bytes;
			val_size -= ctx->val_bytes;
			val += ctx->val_bytes;
		}
		break;

	default:
		return -ENOTSUPP;
	}

	if (ret < 0)
		return ret;

	return 0;
}

static void regmap_smbus_free_context(void *context)
{
	kfree(context);
}

struct regmap_smbus_context *regmap_smbus_gen_context(struct i2c_client *i2c,
				const struct regmap_config *config,
				enum regmap_smbus_transfer_type transfer_type)
{
	struct regmap_smbus_context *ctx;
	int val_bytes = 0;

	if (config->reg_bits != 8 || config->pad_bits != 0)
		return ERR_PTR(-ENOTSUPP);

	switch (transfer_type) {
	case REGMAP_SMBUS_BYTE_TRANSFER:
		if (config->val_bits != 8)
			return ERR_PTR(-EINVAL);

		if (!i2c_check_functionality(i2c->adapter,
					     I2C_FUNC_SMBUS_BYTE_DATA))
			return ERR_PTR(-ENOTSUPP);
		break;

	case REGMAP_SMBUS_WORD_TRANSFER:
		if (config->val_bits != 16)
			return ERR_PTR(-EINVAL);

		if (!i2c_check_functionality(i2c->adapter,
					     I2C_FUNC_SMBUS_WORD_DATA))
			return ERR_PTR(-ENOTSUPP);
		break;

	case REGMAP_SMBUS_BLOCK_TRANSFER:
		if (config->val_bits > (I2C_SMBUS_BLOCK_MAX * 8))
			return ERR_PTR(-EINVAL);

		val_bytes = DIV_ROUND_UP(config->val_bits, 8);
		if (!i2c_check_functionality(i2c->adapter,
					     I2C_FUNC_SMBUS_BLOCK_DATA))
			return ERR_PTR(-ENOTSUPP);
		break;

	case REGMAP_SMBUS_I2C_BLOCK_TRANSFER:
		if (config->val_bits > (I2C_SMBUS_BLOCK_MAX * 8))
			return ERR_PTR(-EINVAL);

		val_bytes = DIV_ROUND_UP(config->val_bits, 8);

		if (!i2c_check_functionality(i2c->adapter,
					     I2C_FUNC_SMBUS_I2C_BLOCK))
			return ERR_PTR(-ENOTSUPP);
		break;

	default:
		return ERR_PTR(-ENOTSUPP);
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->i2c = i2c;
	ctx->transfer_type = transfer_type;
	ctx->val_bytes = val_bytes;

	return ctx;
}

static struct regmap_bus regmap_smbus = {
	.write = regmap_smbus_write,
	.gather_write = regmap_smbus_gather_write,
	.read = regmap_smbus_read,
	.free_context = regmap_smbus_free_context,
};

/**
 * regmap_init_smbus(): Initialise register map
 *
 * @i2c: Device that will be interacted with
 * @config: Configuration for register map
 * @transfer_type: SMBUS transfer type
 *
 * The return value will be an ERR_PTR() on error or a valid pointer to
 * a struct regmap.
 */
struct regmap *regmap_init_smbus(struct i2c_client *i2c,
				 const struct regmap_config *config,
				 enum regmap_smbus_transfer_type transfer_type)
{
	struct regmap_smbus_context *ctx =
		regmap_smbus_gen_context(i2c, config, transfer_type);

	if (IS_ERR(ctx))
		return ERR_PTR(PTR_ERR(ctx));

	return regmap_init(&i2c->dev, &regmap_smbus, ctx, config);
}
EXPORT_SYMBOL_GPL(regmap_init_smbus);

/**
 * devm_regmap_init_smbus(): Initialise managed register map
 *
 * @i2c: Device that will be interacted with
 * @config: Configuration for register map
 * @transfer_type: SMBUS transfer type
 *
 * The return value will be an ERR_PTR() on error or a valid pointer
 * to a struct regmap.  The regmap will be automatically freed by the
 * device management code.
 */
struct regmap *devm_regmap_init_smbus(struct i2c_client *i2c,
				const struct regmap_config *config,
				enum regmap_smbus_transfer_type transfer_type)
{
	struct regmap_smbus_context *ctx =
		regmap_smbus_gen_context(i2c, config, transfer_type);

	if (IS_ERR(ctx))
		return ERR_PTR(PTR_ERR(ctx));

	return devm_regmap_init(&i2c->dev, &regmap_smbus, ctx, config);
}
EXPORT_SYMBOL_GPL(devm_regmap_init_smbus);

MODULE_LICENSE("GPL");
