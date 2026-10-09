#include "config.h"
#include "i2c_bus.h"

/* I2C 主机总线句柄(整个程序仅一个) */
static i2c_master_bus_handle_t s_bus = NULL;

/* 设备句柄缓存: 同一地址只创建一次, 避免重复 add_device */
static i2c_master_dev_handle_t s_dev[I2C_DEV_TABLE_MAX];
static uint8_t                 s_dev_addr[I2C_DEV_TABLE_MAX]; /* 与 s_dev 对应的地址 */
static int                     s_dev_count = 0;

/**
 * 初始化 I2C 主机总线。
 * 引脚/端口/频率等参数均来自 config.h, 重复调用不会重复初始化。
 */
esp_err_t i2c_bus_init(void)
{
    if (s_bus != NULL) {
        return ESP_OK; /* 已初始化过, 直接返回 */
    }
    /* 总线配置: 内部上拉 + 默认时钟源(注意 clk_source 位于匿名 union 内) */
    i2c_master_bus_config_t cfg = {
        .i2c_port             = I2C_MASTER_PORT,
        .sda_io_num           = I2C_MASTER_SDA_IO,
        .scl_io_num           = I2C_MASTER_SCL_IO,
        { .clk_source         = I2C_CLK_SRC_DEFAULT }, /* 匿名 union */
        .glitch_ignore_cnt    = 7,
        .flags.enable_internal_pullup = true,          /* 引脚内部上拉 */
    };
    return i2c_new_master_bus(&cfg, &s_bus);
}

/**
 * 获取 I2C 总线句柄。
 */
i2c_master_bus_handle_t i2c_bus_get_bus(void)
{
    return s_bus;
}

/**
 * 按 7bit 地址获取总线上的设备句柄。
 * 内部做懒创建 + 缓存: 首次访问某地址时 add_device, 之后直接复用;
 * 失败或缓存满时返回 NULL。
 */
i2c_master_dev_handle_t i2c_bus_dev_get(uint8_t addr)
{
    if (s_bus == NULL) {
        return NULL; /* 总线尚未初始化 */
    }
    /* 1. 命中缓存则直接返回 */
    for (int i = 0; i < s_dev_count; i++) {
        if (s_dev_addr[i] == addr) {
            return s_dev[i];
        }
    }
    /* 2. 缓存已满, 无法再创建 */
    if (s_dev_count >= I2C_DEV_TABLE_MAX) {
        return NULL;
    }
    /* 3. 新建设备句柄并写入缓存 */
    i2c_device_config_t dcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, /* 7bit 地址 */
        .device_address  = addr,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(s_bus, &dcfg, &dev) != ESP_OK) {
        return NULL;
    }
    s_dev_addr[s_dev_count] = addr;
    s_dev[s_dev_count]      = dev;
    s_dev_count++;
    return dev;
}

/**
 * 向指定设备的单个寄存器写入一个字节。
 */
esp_err_t i2c_bus_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val }; /* 首字节=寄存器地址, 次字节=写入值 */
    return i2c_master_transmit(dev, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS);
}

/**
 * 从指定设备的寄存器 reg 开始, 连续读取 len 字节到 buf。
 * (依赖传感器的地址自增特性读取连续寄存器块)
 */
esp_err_t i2c_bus_read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, I2C_MASTER_TIMEOUT_MS);
}

/**
 * 复位 I2C 总线: 通过发时钟脉冲让从机释放被拉低的 SDA, 并重新初始化控制器。
 * 场景: 上位机在某次传输中途复位时, 从机可能持续拉低 SDA 导致整条总线"假死",
 *       表现为上电扫描 0 设备, 只有复位总线才能恢复。
 */
esp_err_t i2c_bus_reset(void)
{
    if (s_bus == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_bus_reset(s_bus);
}

/**
 * 扫描 I2C 总线, 将在线设备的 7bit 地址写入 addrs, 返回在线设备数量。
 * 扫描范围 0x03~0x77(避开保留地址)。
 */
int i2c_bus_scan(uint8_t *addrs, int max)
{
    int n = 0;
    for (uint8_t a = 0x03; a <= 0x77 && n < max; a++) {
        if (i2c_master_probe(s_bus, a, I2C_MASTER_TIMEOUT_MS) == ESP_OK) {
            addrs[n++] = a; /* 该地址有应答, 记录 */
        }
    }
    return n;
}
