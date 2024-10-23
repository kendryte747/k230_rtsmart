#include <dfs.h>
#include <dfs_file.h>
#include <dfs_private.h>
#include <unistd.h>

#include <rtthread.h>
#include "usb_osal.h"

#define READ_LEN (150)
static void rd_thread(void *argument)
{
    char *file_name = (char *)argument;
    int fd, len = 0;
    char buf[READ_LEN];

    fd = open(file_name, O_RDONLY);
    if (fd < 0) {
        rt_kprintf("%s open dev fail\n", __func__);
        goto free_buf;
    }

    while (len < READ_LEN) {
        int ret = read(fd, buf + len, READ_LEN - len);
        if (ret < 0) {
            goto clost_fd;
        }
        for (int i = 0; i < ret; i++) {
            rt_kprintf("%c", buf[len + i]);
        }
        rt_kprintf("\n");
        len += ret;
    }

clost_fd:
    close(fd);
free_buf:
    rt_free(file_name);
}

void rd(char *dev_path)
{
    char *file_name;
    usb_osal_thread_t thread;

    file_name = rt_malloc(strlen(dev_path) + 1);
    strcpy(file_name, dev_path);

    thread = usb_osal_thread_create("rd", 2048, 15, rd_thread, file_name);
    if (thread == NULL) {
        rt_free(file_name);
        rt_kprintf("%s fail to create thread\n", __func__);
    }
}
FINSH_FUNCTION_EXPORT(rd, rd data);

#define WRITE_BUF_LEN (32)
void wr(char *file_name, char *data)
{
    int fd, len, i;
    static char buf[WRITE_BUF_LEN];

    if (strlen(data) + 1 > WRITE_BUF_LEN) {
        rt_kprintf("cmd is too long\n");
        goto out;
    }

    fd = open(file_name, O_WRONLY);
    if (fd < 0) {
        rt_kprintf("open dev fail\n");
        goto out;
    }

    for (i = 0; data[i] != '\0'; i++) {
        buf[i] = data[i];
    }

    buf[i++] = 0xd;
    buf[i++] = '\0';

    len = strlen(buf);
    write(fd, buf, len);

    close(fd);
out:
    return;
}
FINSH_FUNCTION_EXPORT(wr, wr data);






int cmd_rd(int argc, char **argv)
{
    if (argc != 2)
    {
        rt_kprintf("Usage: rd /dev/ttyUSB1\n");
        return 0;
    }

    rd(argv[1]);

    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_rd, rd, rd /dev/ttyUSB1);

int cmd_wr(int argc, char **argv)
{
    if (argc != 3)
    {
        rt_kprintf("Usage: wr /dev/ttyUSB1 [data]\n");
        return 0;
    }

    wr(argv[1], argv[2]);

    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_wr, wr, wr /dev/ttyUSB1 [data]);

