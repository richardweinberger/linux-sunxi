/*
 * Copyright (C) 2014 Free Electrons
 *
 * License Terms: GNU General Public License v2
 * Author: Boris BREZILLON <boris.brezillon@free-electrons.com>
 *
 * Allwinner PRCM (Power/Reset/Clock Management) driver
 *
 */

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define SUN6I_APB0_GATES_MAX_SIZE	32
#define SUN6I_AR100_MAX_PARENTS		4

static int sun6i_a31_ar100_mux_clk_register(struct platform_device *pdev)
{
	const char *parents[SUN6I_AR100_MAX_PARENTS];
	struct device_node *np = pdev->dev.of_node;
	const char *clk_name = np->name;
	struct resource *r;
	void __iomem *reg;
	struct clk *clk;
	int nparents;
	int i;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	reg = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	nparents = of_clk_get_parent_count(np);
	if (nparents > SUN6I_AR100_MAX_PARENTS)
		nparents = SUN6I_AR100_MAX_PARENTS;

	for (i = 0; i < nparents; i++)
		parents[i] = of_clk_get_parent_name(np, i);

	of_property_read_string(np, "clock-output-names", &clk_name);

	clk = clk_register_mux(&pdev->dev, clk_name, parents, nparents,
			       CLK_SET_RATE_NO_REPARENT, reg,
			       16, 2, 0, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	return of_clk_add_provider(np, of_clk_src_simple_get, clk);
}

static int sun6i_a31_ar100_clk_register(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const char *clk_name = np->name;
	const char *clk_parent;
	struct resource *r;
	void __iomem *reg;
	struct clk *clk;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	reg = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	clk_parent = of_clk_get_parent_name(np, 0);
	if (!clk_parent)
		return -EINVAL;

	of_property_read_string(np, "clock-output-names", &clk_name);

	clk = clk_register_divider(&pdev->dev, clk_name, clk_parent,
				   0, reg, 4, 2, CLK_DIVIDER_POWER_OF_TWO,
				   NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	return of_clk_add_provider(np, of_clk_src_simple_get, clk);
}

static int sun6i_a31_ar100_div_clk_register(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const char *clk_name = np->name;
	const char *clk_parent;
	struct resource *r;
	void __iomem *reg;
	struct clk *clk;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	reg = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	clk_parent = of_clk_get_parent_name(np, 0);
	if (!clk_parent)
		return -EINVAL;

	of_property_read_string(np, "clock-output-names", &clk_name);

	clk = clk_register_divider(&pdev->dev, clk_name, clk_parent,
				   0, reg, 8, 5, 0, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	return of_clk_add_provider(np, of_clk_src_simple_get, clk);
}

static int sun6i_a31_apb0_clk_register(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const char *clk_name = np->name;
	const char *clk_parent;
	struct resource *r;
	void __iomem *reg;
	struct clk *clk;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	reg = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	clk_parent = of_clk_get_parent_name(np, 0);
	if (!clk_parent)
		return -EINVAL;

	of_property_read_string(np, "clock-output-names", &clk_name);

	clk = clk_register_divider(&pdev->dev, clk_name, clk_parent,
				   0, reg, 0, 2, CLK_DIVIDER_POWER_OF_TWO,
				   NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	return of_clk_add_provider(np, of_clk_src_simple_get, clk);
}

static int sun6i_a31_apb0_gates_clk_register(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct clk_onecell_data *clk_data;
	const char *clk_parent;
	const char *clk_name;
	struct resource *r;
	void __iomem *reg;
	int gate_id;
	int ngates;
	int i;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	reg = devm_ioremap_resource(&pdev->dev, r);
	if (!reg)
		return PTR_ERR(reg);

	clk_parent = of_clk_get_parent_name(np, 0);
	if (!clk_parent)
		return -EINVAL;

	ngates = of_property_count_strings(np, "clock-output-names");
	if (ngates < 0)
		return ngates;

	if (!ngates || ngates > SUN6I_APB0_GATES_MAX_SIZE)
		return -EINVAL;

	clk_data = devm_kzalloc(&pdev->dev, sizeof(struct clk_onecell_data),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->clks = devm_kzalloc(&pdev->dev,
				      SUN6I_APB0_GATES_MAX_SIZE *
				      sizeof(struct clk *),
				      GFP_KERNEL);
	if (!clk_data->clks)
		return -ENOMEM;

	for (i = 0; i < ngates; i++) {
		of_property_read_string_index(np, "clock-output-names",
					      i, &clk_name);

		gate_id = i;
		of_property_read_u32_index(np, "clock-indices", i, &gate_id);

		WARN_ON(gate_id >= SUN6I_APB0_GATES_MAX_SIZE);
		if (gate_id >= SUN6I_APB0_GATES_MAX_SIZE)
			continue;

		clk_data->clks[gate_id] = clk_register_gate(&pdev->dev,
							    clk_name,
							    clk_parent, 0,
							    reg, gate_id,
							    0, NULL);
		WARN_ON(IS_ERR(clk_data->clks[gate_id]));
	}

	clk_data->clk_num = ngates;

	return of_clk_add_provider(np, of_clk_src_onecell_get, clk_data);
}

const struct of_device_id sun6i_a31_prcm_clk_dt_ids[] = {
	{
		.compatible = "allwinner,sun6i-a31-ar100-mux-clk",
		.data = sun6i_a31_ar100_mux_clk_register,
	},
	{
		.compatible = "allwinner,sun6i-a31-ar100-clk",
		.data = sun6i_a31_ar100_clk_register,
	},
	{
		.compatible = "allwinner,sun6i-a31-ar100-div-clk",
		.data = sun6i_a31_ar100_div_clk_register,
	},
	{
		.compatible = "allwinner,sun6i-a31-apb0-clk",
		.data = sun6i_a31_apb0_clk_register,
	},
	{
		.compatible = "allwinner,sun6i-a31-apb0-gates-clk",
		.data = sun6i_a31_apb0_gates_clk_register,
	},
	{ /* sentinel */ }
};

static int sun6i_a31_prcm_clk_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int (*register_func)(struct platform_device *pdev);
	const struct of_device_id *match;

	match = of_match_node(sun6i_a31_prcm_clk_dt_ids, np);
	if (!match)
		return -EINVAL;

	register_func = match->data;
	return register_func(pdev);
}

static struct platform_driver sun6i_a31_prcm_clk_driver = {
	.driver = {
		.name = "sun6i-a31-prcm-clk",
		.owner = THIS_MODULE,
		.of_match_table = sun6i_a31_prcm_clk_dt_ids,
	},
	.probe = sun6i_a31_prcm_clk_probe,
};
module_platform_driver(sun6i_a31_prcm_clk_driver);

MODULE_AUTHOR("Boris BREZILLON <boris.brezillon@free-electrons.com>");
MODULE_DESCRIPTION("Allwinner PRCM clock Driver");
MODULE_LICENSE("GPL v2");
