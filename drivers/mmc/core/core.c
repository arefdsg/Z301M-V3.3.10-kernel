/*
 *  linux/drivers/mmc/core/core.c
 *
 *  Copyright (C) 2003-2004 Russell King, All Rights Reserved.
 *  SD support Copyright (C) 2004 Ian Molton, All Rights Reserved.
 *  Copyright (C) 2005-2008 Pierre Ossman, All Rights Reserved.
 *  MMCv4 support Copyright (C) 2006 Philip Langdale, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/pagemap.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/scatterlist.h>
#include <linux/log2.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/suspend.h>
#include <linux/fault-inject.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/of.h>

#include <trace/events/mmc.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/slot-gpio.h>
#ifdef MTK_BKOPS_IDLE_MAYA
#include <linux/workqueue.h>
#endif
#include <linux/blkdev.h>
#include "core.h"
#include "bus.h"
#include "host.h"
#include "sdio_bus.h"

#include "mmc_ops.h"
#include "sd_ops.h"
#include "sdio_ops.h"
#include "../card/mt_mmc_block.h"

/* If the device is not responding */
#define MMC_CORE_TIMEOUT_MS	(10 * 60 * 1000) /* 10 minute timeout */

/*
 * Background operations can take a long time, depending on the housekeeping
 * operations the card has to perform.
 */
#define MMC_BKOPS_MAX_TIMEOUT	(4 * 60 * 1000) /* max time to wait in ms */

extern int Read_HW_ID(void);
static struct workqueue_struct *workqueue;
static const unsigned freqs[] = { 400000, 300000, 200000, 100000 };

/*
 * Enabling software CRCs on the data blocks can be a significant (30%)
 * performance cost, and for other reasons may not always be desired.
 * So we allow it it to be disabled.
 */
bool use_spi_crc = 1;
module_param(use_spi_crc, bool, 0);
#ifdef MTK_BKOPS_IDLE_MAYA
#define MMC_UPDATE_BKOPS_STATS_HPI(stats)\
	do {\
		spin_lock(&stats.lock);\
		if (stats.enabled)\
			stats.hpi++;\
		spin_unlock(&stats.lock);\
	} while (0)

#define MMC_UPDATE_STATS_BKOPS_SEVERITY_LEVEL(stats, level)\
	do {\
		if (level <= 0 || level > BKOPS_NUM_OF_SEVERITY_LEVELS)\
			break;\
		spin_lock(&stats.lock);\
		if (stats.enabled)\
			stats.bkops_level[level]++;\
		spin_unlock(&stats.lock);\
	} while (0)
#endif
/*
 * Internal function. Schedule delayed work in the MMC work queue.
 */
static int mmc_schedule_delayed_work(struct delayed_work *work,
				     unsigned long delay)
{
	return queue_delayed_work(workqueue, work, delay);
}

/*
 * Internal function. Flush all scheduled work from the MMC work queue.
 */
static void mmc_flush_scheduled_work(void)
{
	flush_workqueue(workqueue);
}

#ifdef CONFIG_FAIL_MMC_REQUEST

/*
 * Internal function. Inject random data errors.
 * If mmc_data is NULL no errors are injected.
 */
static void mmc_should_fail_request(struct mmc_host *host,
				    struct mmc_request *mrq)
{
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	static const int data_errors[] = {
		-ETIMEDOUT,
		-EILSEQ,
		-EIO,
	};

	if (!data)
		return;

	if (cmd->error || data->error ||
	    !should_fail(&host->fail_mmc_request, data->blksz * data->blocks))
		return;

	data->error = data_errors[prandom_u32() % ARRAY_SIZE(data_errors)];
	data->bytes_xfered = (prandom_u32() % (data->bytes_xfered >> 9)) << 9;
}

#else /* CONFIG_FAIL_MMC_REQUEST */

static inline void mmc_should_fail_request(struct mmc_host *host,
					   struct mmc_request *mrq)
{
}

#endif /* CONFIG_FAIL_MMC_REQUEST */

/**
 *	mmc_request_done - finish processing an MMC request
 *	@host: MMC host which completed request
 *	@mrq: MMC request which request
 *
 *	MMC drivers should call this function when they have completed
 *	their processing of a request.
 */
void mmc_request_done(struct mmc_host *host, struct mmc_request *mrq)
{
	struct mmc_command *cmd = mrq->cmd;
	int err = cmd->error;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	int release = 1;

	if (mrq->done && mrq->done == mmc_wait_cmdq_done)
		release = 0;
#endif

#ifdef CONFIG_K42_MMC_RETUNE
	/* Flag re-tuning needed on CRC errors */
	if ((cmd->opcode != MMC_SEND_TUNING_BLOCK &&
	    cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200) &&
	    (err == -EILSEQ || (mrq->sbc && mrq->sbc->error == -EILSEQ) ||
	    (mrq->data && mrq->data->error == -EILSEQ) ||
	    (mrq->stop && mrq->stop->error == -EILSEQ)))
		mmc_retune_needed(host);
#endif

	if (err && cmd->retries && mmc_host_is_spi(host)) {
		if (cmd->resp[0] & R1_SPI_ILLEGAL_COMMAND)
			cmd->retries = 0;
	}

	if (err && cmd->retries && !mmc_card_removed(host->card)) {
		/*
		 * Request starter must handle retries - see
		 * mmc_wait_for_req_done().
		 */
		if (mrq->done)
			mrq->done(mrq);
	} else {
		mmc_should_fail_request(host, mrq);

		led_trigger_event(host->led, LED_OFF);

		pr_debug("%s: req done (CMD%u): %d: %08x %08x %08x %08x\n",
			mmc_hostname(host), cmd->opcode, err,
			cmd->resp[0], cmd->resp[1],
			cmd->resp[2], cmd->resp[3]);

		if (mrq->data) {
			pr_debug("%s:     %d bytes transferred: %d\n",
				mmc_hostname(host),
				mrq->data->bytes_xfered, mrq->data->error);
			trace_mmc_blk_rw_end(cmd->opcode, cmd->arg, mrq->data);
		}

		if (mrq->stop) {
			pr_debug("%s:     (CMD%u): %d: %08x %08x %08x %08x\n",
				mmc_hostname(host), mrq->stop->opcode,
				mrq->stop->error,
				mrq->stop->resp[0], mrq->stop->resp[1],
				mrq->stop->resp[2], mrq->stop->resp[3]);
		}

		if (mrq->done)
			mrq->done(mrq);
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		if (release)
#endif
			mmc_host_clk_release(host);
	}
}

EXPORT_SYMBOL(mmc_request_done);

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
static void mmc_enqueue_queue(struct mmc_host *host, struct mmc_request *mrq)
{
	unsigned long flags;

	if (mrq->cmd->opcode == MMC_READ_REQUESTED_QUEUE ||
		mrq->cmd->opcode == MMC_WRITE_REQUESTED_QUEUE) {
		spin_lock_irqsave(&host->dat_que_lock, flags);
		if (mrq->flags)
			list_add(&mrq->link, &host->dat_que);
		else
			list_add_tail(&mrq->link, &host->dat_que);
		spin_unlock_irqrestore(&host->dat_que_lock, flags);
	} else {
		spin_lock_irqsave(&host->cmd_que_lock, flags);
		if (mrq->flags)
			list_add(&mrq->link, &host->cmd_que);
		else
			list_add_tail(&mrq->link, &host->cmd_que);
		spin_unlock_irqrestore(&host->cmd_que_lock, flags);
	}
}

static void mmc_dequeue_queue(struct mmc_host *host, struct mmc_request *mrq)
{
	unsigned long flags;

	if (mrq->cmd->opcode == MMC_READ_REQUESTED_QUEUE ||
		mrq->cmd->opcode == MMC_WRITE_REQUESTED_QUEUE) {
		spin_lock_irqsave(&host->dat_que_lock, flags);
		list_del_init(&mrq->link);
		spin_unlock_irqrestore(&host->dat_que_lock, flags);
	}
}

static void mmc_clr_dat_mrq_que_flag(struct mmc_host *host)
{
	unsigned int i;

	for (i = 0; i < host->card->ext_csd.cmdq_depth; i++)
		host->data_mrq_queued[i] = false;
}

static void mmc_clr_dat_list(struct mmc_host *host)
{

	unsigned long flags;
	struct mmc_request *mrq = NULL;
	struct mmc_request *mrq_next = NULL;

	spin_lock_irqsave(&host->dat_que_lock, flags);
	list_for_each_entry_safe(mrq, mrq_next, &host->dat_que, link) {
		list_del_init(&mrq->link);
	}
	spin_unlock_irqrestore(&host->dat_que_lock, flags);

	mmc_clr_dat_mrq_que_flag(host);
}

static int mmc_restore_tasks(struct mmc_host *host)
{
	struct mmc_request *mrq_cmd = NULL;
	unsigned int i = 0;
	unsigned int task_id;
	unsigned int tasks;

	tasks = host->task_id_index;

	for (task_id = 0; task_id < host->card->ext_csd.cmdq_depth; task_id++) {
		if (tasks & 0x1) {
			mrq_cmd = host->areq_que[task_id]->mrq_que;
			mmc_enqueue_queue(host, mrq_cmd);
			clear_bit(task_id, &host->task_id_index);
			i++;
		}
		tasks >>= 1;
	}

	return i;
}

static struct mmc_request *mmc_get_cmd_que(struct mmc_host *host)
{
	struct mmc_request *mrq = NULL;

	if (!list_empty(&host->cmd_que)) {
		mrq = list_first_entry(&host->cmd_que,
			struct mmc_request, link);
		list_del_init(&mrq->link);
	}
	return mrq;
}

static struct mmc_request *mmc_get_dat_que(struct mmc_host *host)
{
	struct mmc_request *mrq = NULL;

	if (!list_empty(&host->dat_que)) {
		mrq = list_first_entry(&host->dat_que,
			struct mmc_request, link);
	}
	return mrq;
}

static int mmc_blk_status_check(struct mmc_card *card, unsigned int *status)
{
	struct mmc_command cmd = {0};
	int err, retries = 3;

	cmd.opcode = MMC_SEND_STATUS;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, retries);
	if (err == 0)
		*status = cmd.resp[0];
	else
		pr_err("%s: err %d\n", __func__, err);

	return err;
}

static void mmc_discard_cmdq(struct mmc_host *host)
{
	memset(&host->deq_cmd, 0, sizeof(struct mmc_command));
	memset(&host->deq_mrq, 0, sizeof(struct mmc_request));

	host->deq_cmd.opcode = MMC_CMDQ_TASK_MGMT;
	host->deq_cmd.arg = 1;
	host->deq_cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1B | MMC_CMD_AC;
	host->deq_mrq.data = NULL;
	host->deq_mrq.cmd = &host->deq_cmd;

	host->deq_mrq.done = mmc_wait_cmdq_done;
	host->deq_mrq.host = host;
	host->deq_mrq.cmd->retries = 3;
	host->deq_mrq.cmd->error = 0;
	host->deq_mrq.cmd->mrq = &host->deq_mrq;

	while (1) {
		host->ops->request(host, &host->deq_mrq);

		if (!host->deq_mrq.cmd->error ||
			!host->deq_mrq.cmd->retries)
			break;

		pr_err("%s: req failed (CMD%u): %d, retrying...\n",
			 __func__,
			 host->deq_mrq.cmd->opcode,
			 host->deq_mrq.cmd->error);

		host->deq_mrq.cmd->retries--;
		host->deq_mrq.cmd->error = 0;
	};
}

static void mmc_post_req(struct mmc_host *host, struct mmc_request *mrq, int err);
/*
	check CMDQ QSR
*/
void mmc_do_check(struct mmc_host *host)
{
	memset(&host->que_cmd, 0, sizeof(struct mmc_command));
	memset(&host->que_mrq, 0, sizeof(struct mmc_request));
	host->que_cmd.opcode = MMC_SEND_STATUS;
	host->que_cmd.arg = host->card->rca << 16 | 1 << 15;
	host->que_cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
	host->que_cmd.data = NULL;
	host->que_mrq.cmd = &host->que_cmd;

	host->que_mrq.done = mmc_wait_cmdq_done;
	host->que_mrq.host = host;
	host->que_mrq.cmd->retries = 3;
	host->que_mrq.cmd->error = 0;
	host->que_mrq.cmd->mrq = &host->que_mrq;

	while (1) {
		host->ops->request(host, &host->que_mrq);

		if (!host->que_mrq.cmd->error ||
			!host->que_mrq.cmd->retries)
			break;

		pr_err("%s: req failed (CMD%u): %d, retrying...\n",
			 __func__,
			 host->que_mrq.cmd->opcode,
			 host->que_mrq.cmd->error);

		host->que_mrq.cmd->retries--;
		host->que_mrq.cmd->error = 0;
	};
}

static void mmc_prep_chk_mrq(struct mmc_host *host)
{
	memset(&host->chk_cmd, 0, sizeof(struct mmc_command));
	memset(&host->chk_mrq, 0, sizeof(struct mmc_request));
	host->chk_cmd.opcode = MMC_SEND_STATUS;
	host->chk_cmd.arg = host->card->rca << 16;
	host->chk_cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
	host->chk_cmd.data = NULL;
	host->chk_mrq.cmd = &host->chk_cmd;

	host->chk_mrq.done = mmc_wait_cmdq_done;
	host->chk_mrq.host = host;
	host->chk_mrq.cmd->error = 0;
	host->chk_mrq.cmd->mrq = &host->chk_mrq;
}

static void mmc_prep_areq_que(struct mmc_host *host,
	struct mmc_async_req *areq_que)
{
	areq_que->mrq->done = mmc_wait_cmdq_done;
	areq_que->mrq->host = host;
	areq_que->mrq->cmd->error = 0;
	areq_que->mrq->cmd->mrq = areq_que->mrq;
	areq_que->mrq->cmd->data =
		areq_que->mrq->data;
	areq_que->mrq->data->error = 0;
	areq_que->mrq->data->mrq = areq_que->mrq;
	if (areq_que->mrq->stop) {
		areq_que->mrq->data->stop =
			areq_que->mrq->stop;
		areq_que->mrq->stop->error = 0;
		areq_que->mrq->stop->mrq = areq_que->mrq;
	}
}

/*
	check status register
*/
void mmc_do_status(struct mmc_host *host)
{
	mmc_prep_chk_mrq(host);
	host->ops->request(host, &host->chk_mrq);
}

/*
	send stop command
*/
void mmc_do_stop(struct mmc_host *host)
{
	memset(&host->que_cmd, 0, sizeof(struct mmc_command));
	memset(&host->que_mrq, 0, sizeof(struct mmc_request));
	host->que_cmd.opcode = MMC_STOP_TRANSMISSION;
	host->que_cmd.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	host->que_mrq.cmd = &host->que_cmd;

	host->que_mrq.done = mmc_wait_cmdq_done;
	host->que_mrq.host = host;
	host->que_mrq.cmd->retries = 3;
	host->que_mrq.cmd->error = 0;
	host->que_mrq.cmd->mrq = &host->que_mrq;

	while (1) {
		host->ops->request(host, &host->que_mrq);

		if (!host->que_mrq.cmd->error ||
			!host->que_mrq.cmd->retries)
			break;

		pr_err("%s: req failed (CMD%u): %d, retrying...\n",
			 __func__,
			 host->que_mrq.cmd->opcode,
			 host->que_mrq.cmd->error);

		host->que_mrq.cmd->retries--;
		host->que_mrq.cmd->error = 0;
	};
}

static int mmc_wait_tran(struct mmc_host *host)
{
	u32 status;
	int err;
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(10 * 1000);
	do {
		err = mmc_blk_status_check(host->card, &status);
		if (err) {
			pr_debug("[CQ] check card status error = %d\n", err);
			return 1;
		}

		if ((R1_CURRENT_STATE(status) == R1_STATE_DATA) ||
			(R1_CURRENT_STATE(status) == R1_STATE_RCV))
			mmc_do_stop(host);

		if (time_after(jiffies, timeout)) {
			pr_err("%s: Card stuck in %d state! %s\n", mmc_hostname(host),
				R1_CURRENT_STATE(status), __func__);
			return 1;
		}
	} while (R1_CURRENT_STATE(status) != R1_STATE_TRAN);

	return 0;
}

/*
	check write
*/
static int mmc_check_write(struct mmc_host *host, struct mmc_request *mrq)
{
	int ret = 0;
	u32 status = 0;

	if (mrq->cmd->opcode == MMC_WRITE_REQUESTED_QUEUE) {
		ret = mmc_blk_status_check(host->card, &status);

		if ((status & R1_WP_VIOLATION) || host->wp_error) {
			mrq->data->error = -EROFS;
			pr_err("[%s]: data error = %d, status=0x%x, line:%d\n",
				__func__, mrq->data->error, status, __LINE__);
		}
		mmc_wait_tran(host);
		mrq->data->error = 0;
		host->wp_error = 0;
		atomic_set(&host->cq_w, false);
	}

	return ret;
}

int mmc_run_queue_thread(void *data)
{
	struct mmc_host *host = data;
	struct mmc_request *cmd_mrq = NULL;
	struct mmc_request *dat_mrq = NULL;
	struct mmc_request *done_mrq = NULL;
	unsigned int task_id;
	bool is_err = false;
	bool is_done = false;
	int err;

	pr_err("[CQ] start cmdq thread\n");
	mt_bio_queue_alloc(current);
	while (1) {
		set_current_state(TASK_RUNNING);
		mt_biolog_cmdq_check();

		/* End request stage 1/2 */
		if (atomic_read(&host->cq_rw) || (atomic_read(&host->areq_cnt) <= 1)) {
			if (host->done_mrq) {
				done_mrq = host->done_mrq;
				host->done_mrq = NULL;
			}
		}
		if (done_mrq) {
			if (done_mrq->data->error || done_mrq->cmd->error) {
				mmc_wait_tran(host);
				if (!is_err) {
					is_err = true;
					mmc_discard_cmdq(host);
					mmc_wait_tran(host);
					mmc_clr_dat_list(host);
					atomic_set(&host->cq_rdy_cnt, 0);
				}

#if 0 /*"clk tune" or "data fine tune"*/
				if (host->ops->tuning)
					host->ops->tuning(host, mrq2);
				else
					pr_err("[CQ] no tuning function call\n");

#else /*"data tune"*/
				if (host->ops->execute_tuning) {
					err = host->ops->execute_tuning(host, MMC_SEND_TUNING_BLOCK_HS200);
					if (err)
						pr_err("[CQ] tuning failed\n");
				} else
					pr_err("[CQ] no execute tuning function call\n");
#endif
				host->cur_rw_task = 99;
				task_id = (done_mrq->cmd->arg >> 16) & 0x1f;
				host->ops->request(host, host->areq_que[task_id]->mrq_que);
				atomic_set(&host->cq_wait_rdy, 1);
				done_mrq = NULL;
			}

			atomic_set(&host->cq_rw, false);

			if (done_mrq && !done_mrq->data->error && !done_mrq->cmd->error) {
				task_id = (done_mrq->cmd->arg >> 16) & 0x1f;
				mt_biolog_cmdq_dma_end(task_id);
				mmc_check_write(host, done_mrq);
				host->cur_rw_task = 99;
				is_done = true;

				if (atomic_read(&host->cq_tuning_now) == 1) {
					mmc_restore_tasks(host);
					is_err = false;
					atomic_set(&host->cq_tuning_now, 0);
				}
			}
		}

		/* Send Command 46/47 (DMA) */
		if (!atomic_read(&host->cq_rw)) {
			spin_lock_irq(&host->dat_que_lock);
			dat_mrq = mmc_get_dat_que(host);
			spin_unlock_irq(&host->dat_que_lock);
			if (dat_mrq) {
				BUG_ON(dat_mrq->cmd->opcode != MMC_WRITE_REQUESTED_QUEUE
					&& dat_mrq->cmd->opcode != MMC_READ_REQUESTED_QUEUE);

				if (dat_mrq->cmd->opcode == MMC_WRITE_REQUESTED_QUEUE)
					atomic_set(&host->cq_w, true);

				atomic_set(&host->cq_rw, true);
				task_id = ((dat_mrq->cmd->arg >> 16) & 0x1f);
				mt_biolog_cmdq_dma_start(task_id);
				host->cur_rw_task = task_id;
				host->ops->request(host, dat_mrq);
				atomic_dec(&host->cq_rdy_cnt);
				dat_mrq = NULL;
			}
		}

		/* End request stage 2/2 */
		if (is_done) {
			task_id = (done_mrq->cmd->arg >> 16) & 0x1f;
			mt_biolog_cmdq_isdone_start(task_id, host->areq_que[task_id]->mrq_que);
			err = done_mrq->areq->err_check(host->card, done_mrq->areq);
			mmc_post_req(host, done_mrq, 0);
			mt_biolog_cmdq_isdone_end(task_id);
			mt_biolog_cmdq_check();
			mmc_blk_end_queued_req(host, done_mrq->areq, task_id, err);
			mmc_host_clk_release(host);
			wake_up_interruptible(&host->cmp_que);
			done_mrq = NULL;
			is_done = false;
		}

		/* Send Command 44/45 */
		if (atomic_read(&host->cq_tuning_now) == 0) {
			spin_lock_irq(&host->cmd_que_lock);
			cmd_mrq = mmc_get_cmd_que(host);
			spin_unlock_irq(&host->cmd_que_lock);

			while (cmd_mrq) {
				task_id = ((cmd_mrq->sbc->arg >> 16) & 0x1f);
				mt_biolog_cmdq_queue_task(task_id, cmd_mrq);
				if (host->task_id_index & (1 << task_id)) {
					pr_err("[%s] BUG!!! task_id %d used, task_id_index 0x%08lx, areq_cnt = %d, cq_wait_rdy = %d\n",
						__func__, task_id, host->task_id_index,
						atomic_read(&host->areq_cnt),
						atomic_read(&host->cq_wait_rdy));
					/* mmc_cmd_dump(host); */
					while (1)
						;
				}
				set_bit(task_id, &host->task_id_index);
				host->ops->request(host, cmd_mrq);
				host->task_queue_time[task_id] = jiffies;
				atomic_inc(&host->cq_wait_rdy);
				spin_lock_irq(&host->cmd_que_lock);
				cmd_mrq = mmc_get_cmd_que(host);
				spin_unlock_irq(&host->cmd_que_lock);
			}
		}

		/* Send Command 13' */
		if (atomic_read(&host->cq_wait_rdy) > 0
			&& atomic_read(&host->cq_rdy_cnt) == 0)
			mmc_do_check(host);

		/* Sleep when nothing to do */
		mt_biolog_cmdq_check();
		set_current_state(TASK_INTERRUPTIBLE);
		if (atomic_read(&host->areq_cnt) == 0)
			schedule();
		set_current_state(TASK_RUNNING);
	}
	mt_bio_queue_free(current);
	return 0;
}
#endif
static void __mmc_start_request(struct mmc_host *host, struct mmc_request *mrq)
{
#ifdef CONFIG_K42_MMC_RETUNE
	int err;

	/* Assumes host controller has been runtime resumed by mmc_claim_host */
	err = mmc_retune(host);
	if (err) {
		mrq->cmd->error = err;
		mmc_request_done(host, mrq);
		return;
	}
#endif

	host->ops->request(host, mrq);
}

static int mmc_start_request(struct mmc_host *host, struct mmc_request *mrq)
{
#ifdef CONFIG_MMC_DEBUG
	unsigned int i, sz;
	struct scatterlist *sg;
#endif
	mmc_retune_hold(host);

	if (mmc_card_removed(host->card))
		return -ENOMEDIUM;

	if (mrq->sbc) {
		pr_debug("<%s: starting CMD%u arg %08x flags %08x>\n",
			 mmc_hostname(host), mrq->sbc->opcode,
			 mrq->sbc->arg, mrq->sbc->flags);
	}

	pr_debug("%s: starting CMD%u arg %08x flags %08x\n",
		 mmc_hostname(host), mrq->cmd->opcode,
		 mrq->cmd->arg, mrq->cmd->flags);

	if (mrq->data) {
		pr_debug("%s:     blksz %d blocks %d flags %08x "
			"tsac %d ms nsac %d\n",
			mmc_hostname(host), mrq->data->blksz,
			mrq->data->blocks, mrq->data->flags,
			mrq->data->timeout_ns / 1000000,
			mrq->data->timeout_clks);
	}

	if (mrq->stop) {
		pr_debug("%s:     CMD%u arg %08x flags %08x\n",
			 mmc_hostname(host), mrq->stop->opcode,
			 mrq->stop->arg, mrq->stop->flags);
	}

	WARN_ON(!host->claimed);

	mrq->cmd->error = 0;
	mrq->cmd->mrq = mrq;
	if (mrq->data) {
		BUG_ON(mrq->data->blksz > host->max_blk_size);
		BUG_ON(mrq->data->blocks > host->max_blk_count);
		BUG_ON(mrq->data->blocks * mrq->data->blksz >
			host->max_req_size);

#ifdef CONFIG_MMC_DEBUG
		sz = 0;
		for_each_sg(mrq->data->sg, sg, mrq->data->sg_len, i)
			sz += sg->length;
		BUG_ON(sz != mrq->data->blocks * mrq->data->blksz);
#endif

		mrq->cmd->data = mrq->data;
		mrq->data->error = 0;
		mrq->data->mrq = mrq;
		if (mrq->stop) {
			mrq->data->stop = mrq->stop;
			mrq->stop->error = 0;
			mrq->stop->mrq = mrq;
		}
	}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (host->card && host->card->ext_csd.cmdq_mode_en &&
			mrq->done == mmc_wait_cmdq_done) {
		mmc_enqueue_queue(host, mrq);
		wake_up_process(host->cmdq_thread);
		mmc_host_clk_hold(host);
		led_trigger_event(host->led, LED_FULL);
	} else {
#endif
	mmc_host_clk_hold(host);
	led_trigger_event(host->led, LED_FULL);
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		if (host->card
			&& host->card->ext_csd.cmdq_support
			&& mrq->cmd->opcode != MMC_SEND_STATUS)
			mmc_wait_cmdq_empty(host);
#endif
	__mmc_start_request(host, mrq);
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	}
#endif

	return 0;
}

#ifdef MTK_BKOPS_IDLE_MAYA
void mmc_blk_init_bkops_statistics(struct mmc_card *card)
{
	int i;
	struct mmc_bkops_stats *bkops_stats;

	if (!card)
		return;

	bkops_stats = &card->bkops_info.bkops_stats;

	spin_lock(&bkops_stats->lock);

	for (i = 0; i < BKOPS_NUM_OF_SEVERITY_LEVELS; ++i)
		bkops_stats->bkops_level[i] = 0;

	bkops_stats->suspend = 0;
	bkops_stats->hpi = 0;
	bkops_stats->enabled = true;

	spin_unlock(&bkops_stats->lock);
}
EXPORT_SYMBOL(mmc_blk_init_bkops_statistics);

/**
 *	mmc_start_bkops - start BKOPS for supported cards
 *	@card: MMC card to start BKOPS
 *	@form_exception: A flag to indicate if this function was
 *			 called due to an exception raised by the card
 *
 *	Start background operations whenever requested.
 *	When the urgent BKOPS bit is set in a R1 command response
 *	then background operations should be started immediately.
*/
void mmc_start_bkops(struct mmc_card *card, bool from_exception)
{
	int err;

	BUG_ON(!card);
	if (!card->ext_csd.bkops_en)
		return;

	mmc_claim_host(card->host);

#if 0
	if ((card->bkops_info.cancel_delayed_work) && !from_exception) {
		pr_err("%s: %s: cancel_delayed_work was set, exit\n",
		       mmc_hostname(card->host), __func__);
		card->bkops_info.cancel_delayed_work = false;
		goto out;
	}
#endif

	if (mmc_card_doing_bkops(card)) {
		pr_err("%s: %s: already doing bkops, exit\n", mmc_hostname(card->host), __func__);
		goto out;
	}

	if (from_exception && mmc_card_need_bkops(card)) {
		/*pr_err("%s: %s: from_exception, exit\n",
		   mmc_hostname(card->host), __func__); */
		goto out;
	}

	/*
	 * If the need BKOPS flag is set, there is no need to check if BKOPS
	 * is needed since we already know that it does
	 */
	if (!mmc_card_need_bkops(card)) {
		err = mmc_read_bkops_status(card);
		if (err) {
			pr_err("%s: %s: Failed to read bkops status: %d\n",
			       mmc_hostname(card->host), __func__, err);
			goto out;
		}

		/*pr_err("before:%s: %s: raw_bkops_status=0x%x, from_exception=%d\n",
		   mmc_hostname(card->host), __func__,
		   card->ext_csd.raw_bkops_status,
		   from_exception); */
		if (!card->ext_csd.raw_bkops_status)
			goto out;

		/*pr_err("%s: %s: raw_bkops_status=0x%x, from_exception=%d\n",
		   mmc_hostname(card->host), __func__,
		   card->ext_csd.raw_bkops_status,
		   from_exception); */
	}

	/*
	 * If the function was called due to exception, BKOPS will be performed
	 * after handling the last pending request
	 */
	if (from_exception) {
		pr_err("%s: %s: Level %d from exception, exit",
		       mmc_hostname(card->host), __func__, card->ext_csd.raw_bkops_status);
		mmc_card_set_need_bkops(card);
		goto out;
	}
	/*pr_info("%s: %s: pr_info:Starting bkops\n", mmc_hostname(card->host), __func__);
	   pr_debug("%s: %s: pr_debug:Starting bkops\n", mmc_hostname(card->host), __func__);
	   pr_err("%s: %s: pr_err:Starting bkops\n", mmc_hostname(card->host), __func__);
	   pr_warn("%s: %s: pr_warn:Starting bkops\n", mmc_hostname(card->host), __func__); */

	err = __mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			   EXT_CSD_BKOPS_START, 1, 0, false, true, true);
	if (err) {
		pr_warn("%s: Error %d starting bkops\n", mmc_hostname(card->host), err);
		goto out;
	}
	MMC_UPDATE_STATS_BKOPS_SEVERITY_LEVEL(card->bkops_info.bkops_stats,
					      card->ext_csd.raw_bkops_status);
	mmc_card_clr_need_bkops(card);
	mmc_card_set_doing_bkops(card);
	card->bkops_info.sectors_changed = 0;
out:
	mmc_release_host(card->host);
}
EXPORT_SYMBOL(mmc_start_bkops);

/**
 * mmc_start_idle_time_bkops() - Check the need of non urgent
 * BKOPS
 *
 * @card: MMC card to start BKOPS on
 */
void mmc_start_idle_time_bkops(struct work_struct *work)
{
	struct mmc_card *card = container_of(work, struct mmc_card,
					     bkops_info.dw.work);

	/*
	 * Prevent a race condition between mmc_stop_bkops and the delayed
	 * BKOPS work in case the delayed work is executed on another CPU
	 */
	if (card->bkops_info.cancel_delayed_work)
		return;

	mmc_start_bkops(card, false);
}
EXPORT_SYMBOL(mmc_start_idle_time_bkops);

/**
* mmc_start_delayed_bkops() - Start a delayed work to check for
*      the need of non urgent BKOPS
*
* @card: MMC card to start BKOPS on
*/
void mmc_start_delayed_bkops(struct mmc_card *card)
{
	if (!card || !card->ext_csd.bkops_en || mmc_card_doing_bkops(card))
		return;

	if (card->bkops_info.sectors_changed < card->bkops_info.min_sectors_to_queue_delayed_work)
		return;

	pr_err("%s: %s: queueing delayed_bkops_work\n", mmc_hostname(card->host), __func__);

	/*
	 * cancel_delayed_bkops_work will prevent a race condition between
	 * fetching a request by the mmcqd and the delayed work, in case
	 * it was removed from the queue work but not started yet
	 */
	card->bkops_info.cancel_delayed_work = false;
	queue_delayed_work(system_wq, &card->bkops_info.dw,
			   msecs_to_jiffies(card->bkops_info.delay_ms));
}
EXPORT_SYMBOL(mmc_start_delayed_bkops);
#else
/**
 *	mmc_start_bkops - start BKOPS for supported cards
 *	@card: MMC card to start BKOPS
 *	@form_exception: A flag to indicate if this function was
 *			 called due to an exception raised by the card
 *
 *	Start background operations whenever requested.
 *	When the urgent BKOPS bit is set in a R1 command response
 *	then background operations should be started immediately.
*/
void mmc_start_bkops(struct mmc_card *card, bool from_exception)
{
	int err;
	int timeout;
	bool use_busy_signal;

	BUG_ON(!card);

	if (!card->ext_csd.bkops_en || mmc_card_doing_bkops(card))
		return;

	err = mmc_read_bkops_status(card);
	if (err) {
		pr_err("%s: Failed to read bkops status: %d\n",
		       mmc_hostname(card->host), err);
		return;
	}

	if (!card->ext_csd.raw_bkops_status)
		return;

	if (card->ext_csd.raw_bkops_status < EXT_CSD_BKOPS_LEVEL_2 &&
	    from_exception)
		return;

	mmc_claim_host(card->host);
	if (card->ext_csd.raw_bkops_status >= EXT_CSD_BKOPS_LEVEL_2) {
		timeout = MMC_BKOPS_MAX_TIMEOUT;
		use_busy_signal = true;
	} else {
		timeout = 0;
		use_busy_signal = false;
	}

	mmc_retune_hold(card->host);

	err = __mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_BKOPS_START, 1, timeout,
			use_busy_signal, true, false);
	if (err) {
		pr_warn("%s: Error %d starting bkops\n",
			mmc_hostname(card->host), err);
		mmc_retune_release(card->host);
		goto out;
	}

	/*
	 * For urgent bkops status (LEVEL_2 and more)
	 * bkops executed synchronously, otherwise
	 * the operation is in progress
	 */
	if (!use_busy_signal)
		mmc_card_set_doing_bkops(card);
	else
		mmc_retune_release(card->host);
out:
	mmc_release_host(card->host);
}
EXPORT_SYMBOL(mmc_start_bkops);
#endif
/*
 * mmc_wait_data_done() - done callback for data request
 * @mrq: done data request
 *
 * Wakes up mmc context, passed as a callback to host controller driver
 */
static void mmc_wait_data_done(struct mmc_request *mrq)
{
	struct mmc_context_info *context_info = &mrq->host->context_info;

	context_info->is_done_rcv = true;
	wake_up_interruptible(&context_info->wait);
}

static void mmc_wait_done(struct mmc_request *mrq)
{
	complete(&mrq->completion);
}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
void mmc_wait_cmdq_done(struct mmc_request *mrq)
{
	struct mmc_host *host = mrq->host;
	struct mmc_command *cmd = mrq->cmd;
	int done = 0, task_id;

	if (cmd->opcode == MMC_SEND_STATUS ||
		cmd->opcode == MMC_STOP_TRANSMISSION ||
		cmd->opcode == MMC_CMDQ_TASK_MGMT) {
		/* do nothing */
	} else
		mmc_dequeue_queue(host, mrq);

	/* error - request done */
	if (cmd->error) {
		if ((cmd->opcode == MMC_READ_REQUESTED_QUEUE) ||
			(cmd->opcode == MMC_WRITE_REQUESTED_QUEUE)) {
			atomic_set(&host->cq_tuning_now, 1);
			goto clear_end;
		}
		goto request_end;
	}

	/* data error */
	if (mrq->data && mrq->data->error) {
		atomic_set(&host->cq_tuning_now, 1);
		goto clear_end;
	}

	/* check wp violation */
	if ((cmd->opcode == MMC_SET_QUEUE_CONTEXT) ||
		(cmd->opcode == MMC_QUEUE_READ_ADDRESS)) {
		if (atomic_read(&host->cq_w)) {
			if (cmd->resp[0] & R1_WP_VIOLATION)
				host->wp_error = 1;
		}
	}

	/* cmd13' - check queue ready & enqueue 46/47 */
	if ((cmd->opcode == MMC_SEND_STATUS) && (cmd->arg & (1 << 15))) {
		int i = 0;
		unsigned int resp = cmd->resp[0];

		if (resp == 0)
			goto request_end;

		do {
			if ((resp & 1) && (!host->data_mrq_queued[i])) {
				if (host->cur_rw_task == i) {
					pr_err("[CQ] task %d ready not clear when DMA\n", i);
					resp >>= 1;
					i++;
					continue;
				}
				BUG_ON(!host->areq_que[i]);
				atomic_dec(&host->cq_wait_rdy);
				atomic_inc(&host->cq_rdy_cnt);
				mmc_prep_areq_que(host, host->areq_que[i]);
				mmc_enqueue_queue(host, host->areq_que[i]->mrq);
				host->data_mrq_queued[i] = true;
				host->task_queue_time[i] = 0;
			}
			resp >>= 1;
			i++;
		} while (resp && (i < host->card->ext_csd.cmdq_depth));
		/* Check Task ready time out */
		for (i = 0; i < host->card->ext_csd.cmdq_depth; i++) {
			if ((host->task_id_index & (0x1 << i)) &&
			    (host->task_queue_time[i] != 0)) {
				if (time_after(jiffies,
				    host->task_queue_time[i] + TASK_READY_TMO)) {
					pr_err("[CQ] ERROR Task ready TMO ID: %d ready time is %ld ticks\n", i,
						  (long)(jiffies) - (long)(host->task_queue_time[i]));
			       }
			}
		}

	}

	/* cmd46 - request done */
	if (cmd->opcode == MMC_READ_REQUESTED_QUEUE
		|| cmd->opcode == MMC_WRITE_REQUESTED_QUEUE)
		goto clear_end;

	goto request_end;

clear_end:
	task_id = ((cmd->arg >> 16) & 0x1f);
	clear_bit(task_id, &host->task_id_index);
	host->data_mrq_queued[task_id] = false;
	done = 1;

request_end:
	/* request done when next data transfer */
	if (done) {
		BUG_ON(cmd->opcode != 46 && cmd->opcode != 47);
		BUG_ON(host->done_mrq);
		host->done_mrq = mrq;
	}

	wake_up_interruptible(&host->cmp_que);
}
#endif
/*
 *__mmc_start_data_req() - starts data request
 * @host: MMC host to start the request
 * @mrq: data request to start
 *
 * Sets the done callback to be called when request is completed by the card.
 * Starts data mmc request execution
 */
static int __mmc_start_data_req(struct mmc_host *host, struct mmc_request *mrq)
{
	int err;

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (host->card && host->card->ext_csd.cmdq_mode_en)
		mrq->done = mmc_wait_cmdq_done;
	else
#endif

	mrq->done = mmc_wait_data_done;
	mrq->host = host;

	err = mmc_start_request(host, mrq);
	if (err) {
		mrq->cmd->error = err;
		mmc_wait_data_done(mrq);
	}

	return err;
}

static int __mmc_start_req(struct mmc_host *host, struct mmc_request *mrq)
{
	int err;

	init_completion(&mrq->completion);
	mrq->done = mmc_wait_done;

	err = mmc_start_request(host, mrq);
	if (err) {
		mrq->cmd->error = err;
		complete(&mrq->completion);
	}

	return err;
}

/*
 * mmc_wait_for_data_req_done() - wait for request completed
 * @host: MMC host to prepare the command.
 * @mrq: MMC request to wait for
 *
 * Blocks MMC context till host controller will ack end of data request
 * execution or new request notification arrives from the block layer.
 * Handles command retries.
 *
 * Returns enum mmc_blk_status after checking errors.
 */
static int mmc_wait_for_data_req_done(struct mmc_host *host,
				      struct mmc_request *mrq,
				      struct mmc_async_req *next_req)
{
	struct mmc_command *cmd;
	struct mmc_context_info *context_info = &host->context_info;
	int err;
	unsigned long flags;

	while (1) {
		wait_event_interruptible(context_info->wait,
				(context_info->is_done_rcv ||
				 context_info->is_new_req));
		spin_lock_irqsave(&context_info->lock, flags);
		context_info->is_waiting_last_req = false;
		spin_unlock_irqrestore(&context_info->lock, flags);
		if (context_info->is_done_rcv) {
			context_info->is_done_rcv = false;
			context_info->is_new_req = false;
			cmd = mrq->cmd;

			if (!cmd->error || !cmd->retries ||
			    mmc_card_removed(host->card)) {
				err = host->areq->err_check(host->card,
							    host->areq);
				break; /* return err */
			} else {
				mmc_retune_recheck(host);
				pr_info("%s: req failed (CMD%u): %d, retrying...\n",
					mmc_hostname(host),
					cmd->opcode, cmd->error);
				cmd->retries--;
				cmd->error = 0;
				__mmc_start_request(host, mrq);
				continue; /* wait for done/new event again */
			}
		} else if (context_info->is_new_req) {
			context_info->is_new_req = false;
			if (!next_req)
				return MMC_BLK_NEW_REQUEST;
		}
	}
	mmc_retune_release(host);
	return err;
}

static void mmc_wait_for_req_done(struct mmc_host *host,
				  struct mmc_request *mrq)
{
	struct mmc_command *cmd;

	while (1) {
		wait_for_completion(&mrq->completion);

		cmd = mrq->cmd;

		/*
		 * If host has timed out waiting for the sanitize
		 * to complete, card might be still in programming state
		 * so let's try to bring the card out of programming
		 * state.
		 */
		if (cmd->sanitize_busy && cmd->error == -ETIMEDOUT) {
			if (!mmc_interrupt_hpi(host->card)) {
				pr_warn("%s: %s: Interrupted sanitize\n",
					mmc_hostname(host), __func__);
				cmd->error = 0;
				break;
			} else {
				pr_err("%s: %s: Failed to interrupt sanitize\n",
				       mmc_hostname(host), __func__);
			}
		}
		if (!cmd->error || !cmd->retries ||
		    mmc_card_removed(host->card))
			break;

		mmc_retune_recheck(host);

		pr_debug("%s: req failed (CMD%u): %d, retrying...\n",
			 mmc_hostname(host), cmd->opcode, cmd->error);
		cmd->retries--;
		cmd->error = 0;
		__mmc_start_request(host, mrq);
	}

	mmc_retune_release(host);
}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
static void mmc_wait_for_cmdq_done(struct mmc_host *host)
{
	while ((atomic_read(&host->areq_cnt) != 0) ||
		((host->state) != 0)) {
		wait_event_interruptible(host->cmp_que,
			((atomic_read(&host->areq_cnt) == 0) &&
			((host->state) == 0)));
	}
}

void mmc_wait_cmdq_empty(struct mmc_host *host)
{
	mmc_wait_for_cmdq_done(host);
}
#endif

/**
 *	mmc_pre_req - Prepare for a new request
 *	@host: MMC host to prepare command
 *	@mrq: MMC request to prepare for
 *	@is_first_req: true if there is no previous started request
 *                     that may run in parellel to this call, otherwise false
 *
 *	mmc_pre_req() is called in prior to mmc_start_req() to let
 *	host prepare for the new request. Preparation of a request may be
 *	performed while another request is running on the host.
 */
static void mmc_pre_req(struct mmc_host *host, struct mmc_request *mrq,
		 bool is_first_req)
{
	if (host->ops->pre_req) {
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		if (!host->card->ext_csd.cmdq_mode_en)
#endif
			mmc_host_clk_hold(host);
		host->ops->pre_req(host, mrq, is_first_req);
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		if (!host->card->ext_csd.cmdq_mode_en)
#endif
			mmc_host_clk_release(host);
	}
}

/**
 *	mmc_post_req - Post process a completed request
 *	@host: MMC host to post process command
 *	@mrq: MMC request to post process for
 *	@err: Error, if non zero, clean up any resources made in pre_req
 *
 *	Let the host post process a completed request. Post processing of
 *	a request may be performed while another reuqest is running.
 */
static void mmc_post_req(struct mmc_host *host, struct mmc_request *mrq,
			 int err)
{
	if (host->ops->post_req) {
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		if (!host->card->ext_csd.cmdq_mode_en)
#endif
			mmc_host_clk_hold(host);
		host->ops->post_req(host, mrq, err);
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		if (!host->card->ext_csd.cmdq_mode_en)
#endif
			mmc_host_clk_release(host);
	}
}

/**
 *	mmc_start_req - start a non-blocking request
 *	@host: MMC host to start command
 *	@areq: async request to start
 *	@error: out parameter returns 0 for success, otherwise non zero
 *
 *	Start a new MMC custom command request for a host.
 *	If there is on ongoing async request wait for completion
 *	of that request and start the new one and return.
 *	Does not wait for the new request to complete.
 *
 *      Returns the completed request, NULL in case of none completed.
 *	Wait for the an ongoing request (previoulsy started) to complete and
 *	return the completed request. If there is no ongoing request, NULL
 *	is returned without waiting. NULL is not an error condition.
 */
struct mmc_async_req *mmc_start_req(struct mmc_host *host,
				    struct mmc_async_req *areq, int *error)
{
	int err = 0;
	int start_err = 0;
#if defined(FEATURE_STORAGE_PERF_INDEX)
	unsigned long long time1 = 0;
	unsigned int idx = 0;
#endif
	struct mmc_async_req *data = host->areq;

	/* Prepare a new request */
	if (areq)
		mmc_pre_req(host, areq->mrq, !host->areq);

	if (host->areq) {
		err = mmc_wait_for_data_req_done(host, host->areq->mrq,	areq);
		if (err == MMC_BLK_NEW_REQUEST) {
			if (error)
				*error = err;
			/*
			 * The previous request was not completed,
			 * nothing to return
			 */
			return NULL;
		} else {
			mt_biolog_mmcqd_req_end(host->areq->mrq->data);
		}
		/*
		 * Check BKOPS urgency for each R1 response
		 */
		if (host->card && mmc_card_mmc(host->card) &&
		    ((mmc_resp_type(host->areq->mrq->cmd) == MMC_RSP_R1) ||
		     (mmc_resp_type(host->areq->mrq->cmd) == MMC_RSP_R1B)) &&
		    (host->areq->mrq->cmd->resp[0] & R1_EXCEPTION_EVENT))
			mmc_start_bkops(host->card, true);
	}

	if (!err && areq) {
		trace_mmc_blk_rw_start(areq->mrq->cmd->opcode,
				       areq->mrq->cmd->arg,
				       areq->mrq->data);

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		if (areq->cmdq_en)
			start_err = __mmc_start_data_req(host, areq->mrq_que);
		else
#endif
		start_err = __mmc_start_data_req(host, areq->mrq);
		mt_biolog_mmcqd_req_start(host);
	}


#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	/* skip if this request is in cq mode process */
	if (!(areq && areq->cmdq_en)
		&& host->areq)
#else
	if (host->areq)
#endif
		mmc_post_req(host, host->areq->mrq, 0);

	 /* Cancel a prepared request if it was not started. */
	if ((err || start_err) && areq)
		mmc_post_req(host, areq->mrq, -EINVAL);

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (!(areq && areq->cmdq_en)) {
#endif
		if (err)
			host->areq = NULL;
		else
			host->areq = areq;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	}
#endif
	if (error)
		*error = err;
	return data;
}
EXPORT_SYMBOL(mmc_start_req);

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
int mmc_blk_cmdq_switch(struct mmc_card *card, int enable)
{
	int ret;

	if (!card->ext_csd.cmdq_support)
		return 0;

	if (!enable)
		mmc_wait_cmdq_empty(card->host);

	ret = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
		EXT_CSD_CMDQ_MODE_EN, enable,
		card->ext_csd.generic_cmd6_time);

	if (ret) {
		pr_err("%s: cmdq %s error %d\n",
				mmc_hostname(card->host),
				enable ? "on" : "off",
				ret);
		return ret;
	}

	card->ext_csd.cmdq_mode_en = enable;

	pr_err("%s: set ext_csd.cmdq_mode_en = %d\n",
		mmc_hostname(card->host),
		card->ext_csd.cmdq_mode_en);

	return 0;
}
EXPORT_SYMBOL(mmc_blk_cmdq_switch);
#endif

/**
 *	mmc_wait_for_req - start a request and wait for completion
 *	@host: MMC host to start command
 *	@mrq: MMC request to start
 *
 *	Start a new MMC custom command request for a host, and wait
 *	for the command to complete. Does not attempt to parse the
 *	response.
 */
void mmc_wait_for_req(struct mmc_host *host, struct mmc_request *mrq)
{
	__mmc_start_req(host, mrq);
	mmc_wait_for_req_done(host, mrq);
}
EXPORT_SYMBOL(mmc_wait_for_req);

/**
 *	mmc_interrupt_hpi - Issue for High priority Interrupt
 *	@card: the MMC card associated with the HPI transfer
 *
 *	Issued High Priority Interrupt, and check for card status
 *	until out-of prg-state.
 */
int mmc_interrupt_hpi(struct mmc_card *card)
{
	int err;
	u32 status;
	unsigned long prg_wait;

	BUG_ON(!card);

	if (!card->ext_csd.hpi_en) {
		pr_info("%s: HPI enable bit unset\n", mmc_hostname(card->host));
		return 1;
	}

	mmc_claim_host(card->host);
	err = mmc_send_status(card, &status);
	if (err) {
		pr_err("%s: Get card status fail\n", mmc_hostname(card->host));
		goto out;
	}

	switch (R1_CURRENT_STATE(status)) {
	case R1_STATE_IDLE:
	case R1_STATE_READY:
	case R1_STATE_STBY:
	case R1_STATE_TRAN:
		/*
		 * In idle and transfer states, HPI is not needed and the caller
		 * can issue the next intended command immediately
		 */
		goto out;
	case R1_STATE_PRG:
		break;
	default:
		/* In all other states, it's illegal to issue HPI */
		pr_debug("%s: HPI cannot be sent. Card state=%d\n",
			mmc_hostname(card->host), R1_CURRENT_STATE(status));
		err = -EINVAL;
		goto out;
	}

	err = mmc_send_hpi_cmd(card, &status);
	if (err)
		goto out;

	prg_wait = jiffies + msecs_to_jiffies(card->ext_csd.out_of_int_time);
	do {
		err = mmc_send_status(card, &status);

		if (!err && R1_CURRENT_STATE(status) == R1_STATE_TRAN)
			break;
		if (time_after(jiffies, prg_wait))
			err = -ETIMEDOUT;
	} while (!err);

out:
	mmc_release_host(card->host);
	return err;
}
EXPORT_SYMBOL(mmc_interrupt_hpi);

/**
 *	mmc_wait_for_cmd - start a command and wait for completion
 *	@host: MMC host to start command
 *	@cmd: MMC command to start
 *	@retries: maximum number of retries
 *
 *	Start a new MMC command for a host, and wait for the command
 *	to complete.  Return any error that occurred while the command
 *	was executing.  Do not attempt to parse the response.
 */
int mmc_wait_for_cmd(struct mmc_host *host, struct mmc_command *cmd, int retries)
{
	struct mmc_request mrq = {NULL};

	WARN_ON(!host->claimed);

	memset(cmd->resp, 0, sizeof(cmd->resp));
	cmd->retries = retries;

	mrq.cmd = cmd;
	cmd->data = NULL;

	mmc_wait_for_req(host, &mrq);

	return cmd->error;
}

EXPORT_SYMBOL(mmc_wait_for_cmd);

/**
 *	mmc_stop_bkops - stop ongoing BKOPS
 *	@card: MMC card to check BKOPS
 *
 *	Send HPI command to stop ongoing background operations to
 *	allow rapid servicing of foreground operations, e.g. read/
 *	writes. Wait until the card comes out of the programming state
 *	to avoid errors in servicing read/write requests.
 */
int mmc_stop_bkops(struct mmc_card *card)
{
	int err = 0;

	BUG_ON(!card);
#ifdef MTK_BKOPS_IDLE_MAYA
	/*
	 * Notify the delayed work to be cancelled, in case it was already
	 * removed from the queue, but was not started yet
	 */
	card->bkops_info.cancel_delayed_work = true;
	if (delayed_work_pending(&card->bkops_info.dw))
		cancel_delayed_work_sync(&card->bkops_info.dw);
	if (!mmc_card_doing_bkops(card))
		goto out;
#endif
	err = mmc_interrupt_hpi(card);

	/*
	 * If err is EINVAL, we can't issue an HPI.
	 * It should complete the BKOPS.
	 */
	if (!err || (err == -EINVAL)) {
		mmc_card_clr_doing_bkops(card);
		mmc_retune_release(card->host);
		err = 0;
	}
#ifdef MTK_BKOPS_IDLE_MAYA
	MMC_UPDATE_BKOPS_STATS_HPI(card->bkops_info.bkops_stats);
#endif
	return err;
}
EXPORT_SYMBOL(mmc_stop_bkops);

int mmc_read_bkops_status(struct mmc_card *card)
{
	int err;
	u8 *ext_csd;

	/*
	 * In future work, we should consider storing the entire ext_csd.
	 */
	ext_csd = kmalloc(512, GFP_KERNEL);
	if (!ext_csd) {
		pr_err("%s: could not allocate buffer to receive the ext_csd.\n",
		       mmc_hostname(card->host));
		return -ENOMEM;
	}

	mmc_claim_host(card->host);
	err = mmc_send_ext_csd(card, ext_csd);
	mmc_release_host(card->host);
	if (err)
		goto out;

	card->ext_csd.raw_bkops_status = ext_csd[EXT_CSD_BKOPS_STATUS];
	card->ext_csd.raw_exception_status = ext_csd[EXT_CSD_EXP_EVENTS_STATUS];
out:
	kfree(ext_csd);
	return err;
}
EXPORT_SYMBOL(mmc_read_bkops_status);

/**
 *	mmc_set_data_timeout - set the timeout for a data command
 *	@data: data phase for command
 *	@card: the MMC card associated with the data transfer
 *
 *	Computes the data timeout parameters according to the
 *	correct algorithm given the card type.
 */
void mmc_set_data_timeout(struct mmc_data *data, const struct mmc_card *card)
{
	unsigned int mult;

	/*
	 * SDIO cards only define an upper 1 s limit on access.
	 */
	if (mmc_card_sdio(card)) {
		data->timeout_ns = 1000000000;
		data->timeout_clks = 0;
		return;
	}

	/*
	 * SD cards use a 100 multiplier rather than 10
	 */
	mult = mmc_card_sd(card) ? 100 : 10;

	/*
	 * Scale up the multiplier (and therefore the timeout) by
	 * the r2w factor for writes.
	 */
	if (data->flags & MMC_DATA_WRITE)
		mult <<= card->csd.r2w_factor;

	data->timeout_ns = card->csd.tacc_ns * mult;
	data->timeout_clks = card->csd.tacc_clks * mult;

	/*
	 * SD cards also have an upper limit on the timeout.
	 */
	if (mmc_card_sd(card)) {
		unsigned int timeout_us, limit_us;

		timeout_us = data->timeout_ns / 1000;
		if (mmc_host_clk_rate(card->host))
			timeout_us += data->timeout_clks * 1000 /
				(mmc_host_clk_rate(card->host) / 1000);

		if (data->flags & MMC_DATA_WRITE)
			/*
			 * The MMC spec "It is strongly recommended
			 * for hosts to implement more than 500ms
			 * timeout value even if the card indicates
			 * the 250ms maximum busy length."  Even the
			 * previous value of 300ms is known to be
			 * insufficient for some cards.
			 */
			limit_us = 3000000;
		else
			limit_us = 100000;

		/*
		 * SDHC cards always use these fixed values.
		 */
		if (timeout_us > limit_us || mmc_card_blockaddr(card)) {
			data->timeout_ns = limit_us * 1000;
			data->timeout_clks = 0;
		}

		/* assign limit value if invalid */
		if (timeout_us == 0)
			data->timeout_ns = limit_us * 1000;
	}

	/*
	 * Some cards require longer data read timeout than indicated in CSD.
	 * Address this by setting the read timeout to a "reasonably high"
	 * value. For the cards tested, 600ms has proven enough. If necessary,
	 * this value can be increased if other problematic cards require this.
	 */
	if (mmc_card_long_read_time(card) && data->flags & MMC_DATA_READ) {
		data->timeout_ns = 600000000;
		data->timeout_clks = 0;
	}

	/*
	 * Some cards need very high timeouts if driven in SPI mode.
	 * The worst observed timeout was 900ms after writing a
	 * continuous stream of data until the internal logic
	 * overflowed.
	 */
	if (mmc_host_is_spi(card->host)) {
		if (data->flags & MMC_DATA_WRITE) {
			if (data->timeout_ns < 1000000000)
				data->timeout_ns = 1000000000;	/* 1s */
		} else {
			if (data->timeout_ns < 100000000)
				data->timeout_ns =  100000000;	/* 100ms */
		}
	}
}
EXPORT_SYMBOL(mmc_set_data_timeout);

/**
 *	mmc_align_data_size - pads a transfer size to a more optimal value
 *	@card: the MMC card associated with the data transfer
 *	@sz: original transfer size
 *
 *	Pads the original data size with a number of extra bytes in
 *	order to avoid controller bugs and/or performance hits
 *	(e.g. some controllers revert to PIO for certain sizes).
 *
 *	Returns the improved size, which might be unmodified.
 *
 *	Note that this function is only relevant when issuing a
 *	single scatter gather entry.
 */
unsigned int mmc_align_data_size(struct mmc_card *card, unsigned int sz)
{
	/*
	 * FIXME: We don't have a system for the controller to tell
	 * the core about its problems yet, so for now we just 32-bit
	 * align the size.
	 */
	sz = ((sz + 3) / 4) * 4;

	return sz;
}
EXPORT_SYMBOL(mmc_align_data_size);

/**
 *	__mmc_claim_host - exclusively claim a host
 *	@host: mmc host to claim
 *	@abort: whether or not the operation should be aborted
 *
 *	Claim a host for a set of operations.  If @abort is non null and
 *	dereference a non-zero value then this will return prematurely with
 *	that non-zero value without acquiring the lock.  Returns zero
 *	with the lock held otherwise.
 */
int __mmc_claim_host(struct mmc_host *host, atomic_t *abort)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	int stop;
	bool pm = false;

	might_sleep();

	add_wait_queue(&host->wq, &wait);
	spin_lock_irqsave(&host->lock, flags);
	while (1) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		stop = abort ? atomic_read(abort) : 0;
		if (stop || !host->claimed || host->claimer == current)
			break;
		spin_unlock_irqrestore(&host->lock, flags);
		schedule();
		spin_lock_irqsave(&host->lock, flags);
	}
	set_current_state(TASK_RUNNING);
	if (!stop) {
		host->claimed = 1;
		host->claimer = current;
		host->claim_cnt += 1;
		if (host->claim_cnt == 1)
			pm = true;
	} else
		wake_up(&host->wq);
	spin_unlock_irqrestore(&host->lock, flags);
	remove_wait_queue(&host->wq, &wait);
	if (host->ops->enable && !stop && host->claim_cnt == 1)
		host->ops->enable(host);

	if (pm)
		pm_runtime_get_sync(mmc_dev(host));

	return stop;
}
EXPORT_SYMBOL(__mmc_claim_host);

/**
 *	mmc_release_host - release a host
 *	@host: mmc host to release
 *
 *	Release a MMC host, allowing others to claim the host
 *	for their operations.
 */
void mmc_release_host(struct mmc_host *host)
{
	unsigned long flags;

	WARN_ON(!host->claimed);

	if (host->ops->disable && host->claim_cnt == 1)
		host->ops->disable(host);

	spin_lock_irqsave(&host->lock, flags);
	if (--host->claim_cnt) {
		/* Release for nested claim */
		spin_unlock_irqrestore(&host->lock, flags);
	} else {
		host->claimed = 0;
		host->claimer = NULL;
		spin_unlock_irqrestore(&host->lock, flags);
		wake_up(&host->wq);
		pm_runtime_mark_last_busy(mmc_dev(host));
		pm_runtime_put_autosuspend(mmc_dev(host));
	}
}
EXPORT_SYMBOL(mmc_release_host);

/*
 * This is a helper function, which fetches a runtime pm reference for the
 * card device and also claims the host.
 */
void mmc_get_card(struct mmc_card *card)
{
	pm_runtime_get_sync(&card->dev);
	mmc_claim_host(card->host);
}
EXPORT_SYMBOL(mmc_get_card);

/*
 * This is a helper function, which releases the host and drops the runtime
 * pm reference for the card device.
 */
void mmc_put_card(struct mmc_card *card)
{
	mmc_release_host(card->host);
	pm_runtime_mark_last_busy(&card->dev);
	pm_runtime_put_autosuspend(&card->dev);
}
EXPORT_SYMBOL(mmc_put_card);

/*
 * Internal function that does the actual ios call to the host driver,
 * optionally printing some debug output.
 */
static inline void mmc_set_ios(struct mmc_host *host)
{
	struct mmc_ios *ios = &host->ios;

	pr_debug("%s: clock %uHz busmode %u powermode %u cs %u Vdd %u "
		"width %u timing %u\n",
		 mmc_hostname(host), ios->clock, ios->bus_mode,
		 ios->power_mode, ios->chip_select, ios->vdd,
		 ios->bus_width, ios->timing);

	if (ios->clock > 0)
		mmc_set_ungated(host);
	host->ops->set_ios(host, ios);
}

/*
 * Control chip select pin on a host.
 */
void mmc_set_chip_select(struct mmc_host *host, int mode)
{
	mmc_host_clk_hold(host);
	host->ios.chip_select = mode;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

/*
 * Sets the host clock to the highest possible frequency that
 * is below "hz".
 */
static void __mmc_set_clock(struct mmc_host *host, unsigned int hz)
{
	WARN_ON(hz && hz < host->f_min);

	if (hz > host->f_max)
		hz = host->f_max;

	host->ios.clock = hz;
	mmc_set_ios(host);
}

void mmc_set_clock(struct mmc_host *host, unsigned int hz)
{
	mmc_host_clk_hold(host);
	__mmc_set_clock(host, hz);
	mmc_host_clk_release(host);
}

#ifdef CONFIG_MMC_CLKGATE
/*
 * This gates the clock by setting it to 0 Hz.
 */
void mmc_gate_clock(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->clk_lock, flags);
	host->clk_old = host->ios.clock;
	host->ios.clock = 0;
	host->clk_gated = true;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	mmc_set_ios(host);
}

/*
 * This restores the clock from gating by using the cached
 * clock value.
 */
void mmc_ungate_clock(struct mmc_host *host)
{
	/*
	 * We should previously have gated the clock, so the clock shall
	 * be 0 here! The clock may however be 0 during initialization,
	 * when some request operations are performed before setting
	 * the frequency. When ungate is requested in that situation
	 * we just ignore the call.
	 */
	if (host->clk_old) {
		BUG_ON(host->ios.clock);
		/* This call will also set host->clk_gated to false */
		__mmc_set_clock(host, host->clk_old);
	}
}

void mmc_set_ungated(struct mmc_host *host)
{
	unsigned long flags;

	/*
	 * We've been given a new frequency while the clock is gated,
	 * so make sure we regard this as ungating it.
	 */
	spin_lock_irqsave(&host->clk_lock, flags);
	host->clk_gated = false;
	spin_unlock_irqrestore(&host->clk_lock, flags);
}

#else
void mmc_set_ungated(struct mmc_host *host)
{
}
#endif

int mmc_execute_tuning(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	u32 opcode;
	int err;

	if (!host->ops->execute_tuning)
		return 0;

	if (mmc_card_mmc(card))
		opcode = MMC_SEND_TUNING_BLOCK_HS200;
	else
		opcode = MMC_SEND_TUNING_BLOCK;

	mmc_host_clk_hold(host);
	err = host->ops->execute_tuning(host, opcode);
	mmc_host_clk_release(host);

	if (err)
		pr_err("%s: tuning execution failed\n", mmc_hostname(host));
#ifdef CONFIG_K42_MMC_RETUNE
	else
		mmc_retune_enable(host);
#endif

	return err;
}

/*
 * Change the bus mode (open drain/push-pull) of a host.
 */
void mmc_set_bus_mode(struct mmc_host *host, unsigned int mode)
{
	mmc_host_clk_hold(host);
	host->ios.bus_mode = mode;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

/*
 * Change data bus width of a host.
 */
void mmc_set_bus_width(struct mmc_host *host, unsigned int width)
{
	mmc_host_clk_hold(host);
	host->ios.bus_width = width;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

/*
 * Set initial state after a power cycle or a hw_reset.
 */
void mmc_set_initial_state(struct mmc_host *host)
{
	mmc_retune_disable(host);

	if (mmc_host_is_spi(host))
		host->ios.chip_select = MMC_CS_HIGH;
	else
		host->ios.chip_select = MMC_CS_DONTCARE;
	host->ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	host->ios.bus_width = MMC_BUS_WIDTH_1;
	host->ios.timing = MMC_TIMING_LEGACY;

	mmc_set_ios(host);
}

/**
 * mmc_vdd_to_ocrbitnum - Convert a voltage to the OCR bit number
 * @vdd:	voltage (mV)
 * @low_bits:	prefer low bits in boundary cases
 *
 * This function returns the OCR bit number according to the provided @vdd
 * value. If conversion is not possible a negative errno value returned.
 *
 * Depending on the @low_bits flag the function prefers low or high OCR bits
 * on boundary voltages. For example,
 * with @low_bits = true, 3300 mV translates to ilog2(MMC_VDD_32_33);
 * with @low_bits = false, 3300 mV translates to ilog2(MMC_VDD_33_34);
 *
 * Any value in the [1951:1999] range translates to the ilog2(MMC_VDD_20_21).
 */
static int mmc_vdd_to_ocrbitnum(int vdd, bool low_bits)
{
	const int max_bit = ilog2(MMC_VDD_35_36);
	int bit;

	if (vdd < 1650 || vdd > 3600)
		return -EINVAL;

	if (vdd >= 1650 && vdd <= 1950)
		return ilog2(MMC_VDD_165_195);

	if (low_bits)
		vdd -= 1;

	/* Base 2000 mV, step 100 mV, bit's base 8. */
	bit = (vdd - 2000) / 100 + 8;
	if (bit > max_bit)
		return max_bit;
	return bit;
}

/**
 * mmc_vddrange_to_ocrmask - Convert a voltage range to the OCR mask
 * @vdd_min:	minimum voltage value (mV)
 * @vdd_max:	maximum voltage value (mV)
 *
 * This function returns the OCR mask bits according to the provided @vdd_min
 * and @vdd_max values. If conversion is not possible the function returns 0.
 *
 * Notes wrt boundary cases:
 * This function sets the OCR bits for all boundary voltages, for example
 * [3300:3400] range is translated to MMC_VDD_32_33 | MMC_VDD_33_34 |
 * MMC_VDD_34_35 mask.
 */
u32 mmc_vddrange_to_ocrmask(int vdd_min, int vdd_max)
{
	u32 mask = 0;

	if (vdd_max < vdd_min)
		return 0;

	/* Prefer high bits for the boundary vdd_max values. */
	vdd_max = mmc_vdd_to_ocrbitnum(vdd_max, false);
	if (vdd_max < 0)
		return 0;

	/* Prefer low bits for the boundary vdd_min values. */
	vdd_min = mmc_vdd_to_ocrbitnum(vdd_min, true);
	if (vdd_min < 0)
		return 0;

	/* Fill the mask, from max bit to min bit. */
	while (vdd_max >= vdd_min)
		mask |= 1 << vdd_max--;

	return mask;
}
EXPORT_SYMBOL(mmc_vddrange_to_ocrmask);

#ifdef CONFIG_OF

/**
 * mmc_of_parse_voltage - return mask of supported voltages
 * @np: The device node need to be parsed.
 * @mask: mask of voltages available for MMC/SD/SDIO
 *
 * 1. Return zero on success.
 * 2. Return negative errno: voltage-range is invalid.
 */
int mmc_of_parse_voltage(struct device_node *np, u32 *mask)
{
	const u32 *voltage_ranges;
	int num_ranges, i;

	voltage_ranges = of_get_property(np, "voltage-ranges", &num_ranges);
	num_ranges = num_ranges / sizeof(*voltage_ranges) / 2;
	if (!voltage_ranges || !num_ranges) {
		pr_info("%s: voltage-ranges unspecified\n", np->full_name);
		return -EINVAL;
	}

	for (i = 0; i < num_ranges; i++) {
		const int j = i * 2;
		u32 ocr_mask;

		ocr_mask = mmc_vddrange_to_ocrmask(
				be32_to_cpu(voltage_ranges[j]),
				be32_to_cpu(voltage_ranges[j + 1]));
		if (!ocr_mask) {
			pr_err("%s: voltage-range #%d is invalid\n",
				np->full_name, i);
			return -EINVAL;
		}
		*mask |= ocr_mask;
	}

	return 0;
}
EXPORT_SYMBOL(mmc_of_parse_voltage);

#endif /* CONFIG_OF */

#ifdef CONFIG_REGULATOR

/**
 * mmc_regulator_get_ocrmask - return mask of supported voltages
 * @supply: regulator to use
 *
 * This returns either a negative errno, or a mask of voltages that
 * can be provided to MMC/SD/SDIO devices using the specified voltage
 * regulator.  This would normally be called before registering the
 * MMC host adapter.
 */
int mmc_regulator_get_ocrmask(struct regulator *supply)
{
	int			result = 0;
	int			count;
	int			i;
	int			vdd_uV;
	int			vdd_mV;

	count = regulator_count_voltages(supply);
	if (count < 0)
		return count;

	for (i = 0; i < count; i++) {
		vdd_uV = regulator_list_voltage(supply, i);
		if (vdd_uV <= 0)
			continue;

		vdd_mV = vdd_uV / 1000;
		result |= mmc_vddrange_to_ocrmask(vdd_mV, vdd_mV);
	}

	if (!result) {
		vdd_uV = regulator_get_voltage(supply);
		if (vdd_uV <= 0)
			return vdd_uV;

		vdd_mV = vdd_uV / 1000;
		result = mmc_vddrange_to_ocrmask(vdd_mV, vdd_mV);
	}

	return result;
}
EXPORT_SYMBOL_GPL(mmc_regulator_get_ocrmask);

/**
 * mmc_regulator_set_ocr - set regulator to match host->ios voltage
 * @mmc: the host to regulate
 * @supply: regulator to use
 * @vdd_bit: zero for power off, else a bit number (host->ios.vdd)
 *
 * Returns zero on success, else negative errno.
 *
 * MMC host drivers may use this to enable or disable a regulator using
 * a particular supply voltage.  This would normally be called from the
 * set_ios() method.
 */
int mmc_regulator_set_ocr(struct mmc_host *mmc,
			struct regulator *supply,
			unsigned short vdd_bit)
{
	int			result = 0;
	int			min_uV, max_uV;

	if (vdd_bit) {
		int		tmp;

		/*
		 * REVISIT mmc_vddrange_to_ocrmask() may have set some
		 * bits this regulator doesn't quite support ... don't
		 * be too picky, most cards and regulators are OK with
		 * a 0.1V range goof (it's a small error percentage).
		 */
		tmp = vdd_bit - ilog2(MMC_VDD_165_195);
		if (tmp == 0) {
			min_uV = 1650 * 1000;
			max_uV = 1950 * 1000;
		} else {
			min_uV = 1900 * 1000 + tmp * 100 * 1000;
			max_uV = min_uV + 100 * 1000;
		}

		result = regulator_set_voltage(supply, min_uV, max_uV);
		if (result == 0 && !mmc->regulator_enabled) {
			result = regulator_enable(supply);
			if (!result)
				mmc->regulator_enabled = true;
		}
	} else if (mmc->regulator_enabled) {
		result = regulator_disable(supply);
		if (result == 0)
			mmc->regulator_enabled = false;
	}

	if (result)
		dev_err(mmc_dev(mmc),
			"could not set regulator OCR (%d)\n", result);
	return result;
}
EXPORT_SYMBOL_GPL(mmc_regulator_set_ocr);

#endif /* CONFIG_REGULATOR */

int mmc_regulator_get_supply(struct mmc_host *mmc)
{
	struct device *dev = mmc_dev(mmc);
	int ret;
	char *vmmc_supply;

#if defined(CONFIG_Z300M) && !defined(CONFIG_Z301M) && !defined(CONFIG_Z301MF)
	int HW_ID = Read_HW_ID();
	if (!strncmp(mmc_hostname(mmc), "mmc1", 4)){
		if(HW_ID != 0 ) //ER, PR, MP
			vmmc_supply = "vmmc";
		else //SR
			vmmc_supply = "sdldo";
	}
	else //mmc0
		vmmc_supply = "vmmc";
#else
	vmmc_supply = "vmmc";
#endif

	mmc->supply.vmmc = devm_regulator_get_optional(dev, vmmc_supply);
	mmc->supply.vqmmc = devm_regulator_get_optional(dev, "vqmmc");

	if (IS_ERR(mmc->supply.vmmc)) {
		if (PTR_ERR(mmc->supply.vmmc) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_info(dev, "No vmmc regulator found\n");
	} else {
		ret = mmc_regulator_get_ocrmask(mmc->supply.vmmc);
		if (ret > 0)
			mmc->ocr_avail = ret;
		else
			dev_warn(dev, "Failed getting OCR mask: %d\n", ret);
	}

	if (IS_ERR(mmc->supply.vqmmc)) {
		if (PTR_ERR(mmc->supply.vqmmc) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_info(dev, "No vqmmc regulator found\n");
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mmc_regulator_get_supply);

/*
 * Mask off any voltages we don't support and select
 * the lowest voltage
 */
u32 mmc_select_voltage(struct mmc_host *host, u32 ocr)
{
	int bit;

	/*
	 * Sanity check the voltages that the card claims to
	 * support.
	 */
	if (ocr & 0x7F) {
		dev_warn(mmc_dev(host),
		"card claims to support voltages below defined range\n");
		ocr &= ~0x7F;
	}

	ocr &= host->ocr_avail;
	if (!ocr) {
		dev_warn(mmc_dev(host), "no support for card's volts\n");
		return 0;
	}

	if (host->caps2 & MMC_CAP2_FULL_PWR_CYCLE) {
		bit = ffs(ocr) - 1;
		ocr &= 3 << bit;
		mmc_power_cycle(host, ocr);
	} else {
		bit = fls(ocr) - 1;
		ocr &= 3 << bit;
		if (bit != host->ios.vdd)
			dev_warn(mmc_dev(host), "exceeding card's volts\n");
	}

	return ocr;
}

int __mmc_set_signal_voltage(struct mmc_host *host, int signal_voltage)
{
	int err = 0;
	int old_signal_voltage = host->ios.signal_voltage;

	host->ios.signal_voltage = signal_voltage;
	if (host->ops->start_signal_voltage_switch) {
		mmc_host_clk_hold(host);
		err = host->ops->start_signal_voltage_switch(host, &host->ios);
		mmc_host_clk_release(host);
	}

	if (err)
		host->ios.signal_voltage = old_signal_voltage;

	return err;

}

int mmc_set_signal_voltage(struct mmc_host *host, int signal_voltage, u32 ocr)
{
	struct mmc_command cmd = {0};
	int err = 0;
	u32 clock;

	BUG_ON(!host);

	/*
	 * Send CMD11 only if the request is to switch the card to
	 * 1.8V signalling.
	 */
	if (signal_voltage == MMC_SIGNAL_VOLTAGE_330)
		return __mmc_set_signal_voltage(host, signal_voltage);

	/*
	 * If we cannot switch voltages, return failure so the caller
	 * can continue without UHS mode
	 */
	if (!host->ops->start_signal_voltage_switch)
		return -EPERM;
	if (!host->ops->card_busy)
		pr_warn("%s: cannot verify signal voltage switch\n",
			mmc_hostname(host));

	cmd.opcode = SD_SWITCH_VOLTAGE;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(host, &cmd, 0);
	if (err)
		return err;

	if (!mmc_host_is_spi(host) && (cmd.resp[0] & R1_ERROR))
		return -EIO;

	mmc_host_clk_hold(host);
	/*
	 * The card should drive cmd and dat[0:3] low immediately
	 * after the response of cmd11, but wait 1 ms to be sure
	 */
	mmc_delay(1);
	if (host->ops->card_busy && !host->ops->card_busy(host)) {
		err = -EAGAIN;
		goto power_cycle;
	}
	/*
	 * During a signal voltage level switch, the clock must be gated
	 * for 5 ms according to the SD spec
	 */
	clock = host->ios.clock;
	host->ios.clock = 0;
	mmc_set_ios(host);

	if (__mmc_set_signal_voltage(host, signal_voltage)) {
		/*
		 * Voltages may not have been switched, but we've already
		 * sent CMD11, so a power cycle is required anyway
		 */
		err = -EAGAIN;
		goto power_cycle;
	}

	/* Keep clock gated for at least 5 ms */
	mmc_delay(5);
	host->ios.clock = clock;
	mmc_set_ios(host);

	/* Wait for at least 1 ms according to spec */
	mmc_delay(1);

	/*
	 * Failure to switch is indicated by the card holding
	 * dat[0:3] low
	 */
	if (host->ops->card_busy && host->ops->card_busy(host))
		err = -EAGAIN;

power_cycle:
	if (err) {
		pr_debug("%s: Signal voltage switch failed, "
			"power cycling card\n", mmc_hostname(host));
		mmc_power_cycle(host, ocr);
	}

	mmc_host_clk_release(host);

	return err;
}

/*
 * Select timing parameters for host.
 */
void mmc_set_timing(struct mmc_host *host, unsigned int timing)
{
	mmc_host_clk_hold(host);
	host->ios.timing = timing;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

/*
 * Select appropriate driver type for host.
 */
void mmc_set_driver_type(struct mmc_host *host, unsigned int drv_type)
{
	mmc_host_clk_hold(host);
	host->ios.drv_type = drv_type;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

/*
 * Apply power to the MMC stack.  This is a two-stage process.
 * First, we enable power to the card without the clock running.
 * We then wait a bit for the power to stabilise.  Finally,
 * enable the bus drivers and clock to the card.
 *
 * We must _NOT_ enable the clock prior to power stablising.
 *
 * If a host does all the power sequencing itself, ignore the
 * initial MMC_POWER_UP stage.
 */
void mmc_power_up(struct mmc_host *host, u32 ocr)
{
	if (host->ios.power_mode == MMC_POWER_ON)
		return;

	mmc_host_clk_hold(host);

	host->ios.vdd = fls(ocr) - 1;
	host->ios.power_mode = MMC_POWER_UP;
	/* Set initial state and call mmc_set_ios */
	mmc_set_initial_state(host);

	/* Try to set signal voltage to 3.3V but fall back to 1.8v or 1.2v */
	if (__mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_330) == 0)
		dev_dbg(mmc_dev(host), "Initial signal voltage of 3.3v\n");
	else if (__mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_180) == 0)
		dev_dbg(mmc_dev(host), "Initial signal voltage of 1.8v\n");
	else if (__mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_120) == 0)
		dev_dbg(mmc_dev(host), "Initial signal voltage of 1.2v\n");

	/*
	 * This delay should be sufficient to allow the power supply
	 * to reach the minimum voltage.
	 */
	mmc_delay(10);

	host->ios.clock = host->f_init;

	host->ios.power_mode = MMC_POWER_ON;
	mmc_set_ios(host);

	/*
	 * This delay must be at least 74 clock sizes, or 1 ms, or the
	 * time required to reach a stable voltage.
	 */
	mmc_delay(10);

	mmc_host_clk_release(host);
}

void mmc_power_off(struct mmc_host *host)
{
	if (host->ios.power_mode == MMC_POWER_OFF)
		return;

	mmc_host_clk_hold(host);

	host->ios.clock = 0;
	host->ios.vdd = 0;

	host->ios.power_mode = MMC_POWER_OFF;
	/* Set initial state and call mmc_set_ios */
	mmc_set_initial_state(host);

	/*
	 * Some configurations, such as the 802.11 SDIO card in the OLPC
	 * XO-1.5, require a short delay after poweroff before the card
	 * can be successfully turned on again.
	 */
	mmc_delay(1);

	mmc_host_clk_release(host);
}

void mmc_power_cycle(struct mmc_host *host, u32 ocr)
{
	mmc_power_off(host);
	/* Wait at least 1 ms according to SD spec */
	mmc_delay(1);
	mmc_power_up(host, ocr);
}

/*
 * Cleanup when the last reference to the bus operator is dropped.
 */
static void __mmc_release_bus(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(host->bus_refs);
	BUG_ON(!host->bus_dead);

	host->bus_ops = NULL;
}

/*
 * Increase reference count of bus operator
 */
static inline void mmc_bus_get(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->bus_refs++;
	spin_unlock_irqrestore(&host->lock, flags);
}

/*
 * Decrease reference count of bus operator and free it if
 * it is the last reference.
 */
static inline void mmc_bus_put(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->bus_refs--;
	if ((host->bus_refs == 0) && host->bus_ops)
		__mmc_release_bus(host);
	spin_unlock_irqrestore(&host->lock, flags);
}

/*
 * Assign a mmc bus handler to a host. Only one bus handler may control a
 * host at any given time.
 */
void mmc_attach_bus(struct mmc_host *host, const struct mmc_bus_ops *ops)
{
	unsigned long flags;

	BUG_ON(!host);
	BUG_ON(!ops);

	WARN_ON(!host->claimed);

	spin_lock_irqsave(&host->lock, flags);

	BUG_ON(host->bus_ops);
	BUG_ON(host->bus_refs);

	host->bus_ops = ops;
	host->bus_refs = 1;
	host->bus_dead = 0;

	spin_unlock_irqrestore(&host->lock, flags);
}

/*
 * Remove the current bus handler from a host.
 */
void mmc_detach_bus(struct mmc_host *host)
{
	unsigned long flags;

	BUG_ON(!host);

	WARN_ON(!host->claimed);
	WARN_ON(!host->bus_ops);

	spin_lock_irqsave(&host->lock, flags);

	host->bus_dead = 1;

	spin_unlock_irqrestore(&host->lock, flags);

	mmc_bus_put(host);
}

static void _mmc_detect_change(struct mmc_host *host, unsigned long delay,
				bool cd_irq)
{
#ifdef CONFIG_MMC_DEBUG
	unsigned long flags;
	spin_lock_irqsave(&host->lock, flags);
	WARN_ON(host->removed);
	spin_unlock_irqrestore(&host->lock, flags);
#endif

	/*
	 * If the device is configured as wakeup, we prevent a new sleep for
	 * 5 s to give provision for user space to consume the event.
	 */
	if (cd_irq && !(host->caps & MMC_CAP_NEEDS_POLL) &&
		device_can_wakeup(mmc_dev(host)))
		pm_wakeup_event(mmc_dev(host), 5000);

	host->detect_change = 1;
	mmc_schedule_delayed_work(&host->detect, delay);
}

/**
 *	mmc_detect_change - process change of state on a MMC socket
 *	@host: host which changed state.
 *	@delay: optional delay to wait before detection (jiffies)
 *
 *	MMC drivers should call this when they detect a card has been
 *	inserted or removed. The MMC layer will confirm that any
 *	present card is still functional, and initialize any newly
 *	inserted.
 */
void mmc_detect_change(struct mmc_host *host, unsigned long delay)
{
	_mmc_detect_change(host, delay, true);
}
EXPORT_SYMBOL(mmc_detect_change);

void mmc_init_erase(struct mmc_card *card)
{
	unsigned int sz;

	if (is_power_of_2(card->erase_size))
		card->erase_shift = ffs(card->erase_size) - 1;
	else
		card->erase_shift = 0;

	/*
	 * It is possible to erase an arbitrarily large area of an SD or MMC
	 * card.  That is not desirable because it can take a long time
	 * (minutes) potentially delaying more important I/O, and also the
	 * timeout calculations become increasingly hugely over-estimated.
	 * Consequently, 'pref_erase' is defined as a guide to limit erases
	 * to that size and alignment.
	 *
	 * For SD cards that define Allocation Unit size, limit erases to one
	 * Allocation Unit at a time.  For MMC cards that define High Capacity
	 * Erase Size, whether it is switched on or not, limit to that size.
	 * Otherwise just have a stab at a good value.  For modern cards it
	 * will end up being 4MiB.  Note that if the value is too small, it
	 * can end up taking longer to erase.
	 */
	if (mmc_card_sd(card) && card->ssr.au) {
		card->pref_erase = card->ssr.au;
		card->erase_shift = ffs(card->ssr.au) - 1;
	} else if (card->ext_csd.hc_erase_size) {
		card->pref_erase = card->ext_csd.hc_erase_size;
	} else if (card->erase_size) {
		sz = (card->csd.capacity << (card->csd.read_blkbits - 9)) >> 11;
		if (sz < 128)
			card->pref_erase = 512 * 1024 / 512;
		else if (sz < 512)
			card->pref_erase = 1024 * 1024 / 512;
		else if (sz < 1024)
			card->pref_erase = 2 * 1024 * 1024 / 512;
		else
			card->pref_erase = 4 * 1024 * 1024 / 512;
		if (card->pref_erase < card->erase_size)
			card->pref_erase = card->erase_size;
		else {
			sz = card->pref_erase % card->erase_size;
			if (sz)
				card->pref_erase += card->erase_size - sz;
		}
	} else
		card->pref_erase = 0;
}

static unsigned int mmc_mmc_erase_timeout(struct mmc_card *card,
				          unsigned int arg, unsigned int qty)
{
	unsigned int erase_timeout;

	if (arg == MMC_DISCARD_ARG ||
	    (arg == MMC_TRIM_ARG && card->ext_csd.rev >= 6)) {
		erase_timeout = card->ext_csd.trim_timeout;
	} else if (card->ext_csd.erase_group_def & 1) {
		/* High Capacity Erase Group Size uses HC timeouts */
		if (arg == MMC_TRIM_ARG)
			erase_timeout = card->ext_csd.trim_timeout;
		else
			erase_timeout = card->ext_csd.hc_erase_timeout;
	} else {
		/* CSD Erase Group Size uses write timeout */
		unsigned int mult = (10 << card->csd.r2w_factor);
		unsigned int timeout_clks = card->csd.tacc_clks * mult;
		unsigned int timeout_us;

		/* Avoid overflow: e.g. tacc_ns=80000000 mult=1280 */
		if (card->csd.tacc_ns < 1000000)
			timeout_us = (card->csd.tacc_ns * mult) / 1000;
		else
			timeout_us = (card->csd.tacc_ns / 1000) * mult;

		/*
		 * ios.clock is only a target.  The real clock rate might be
		 * less but not that much less, so fudge it by multiplying by 2.
		 */
		timeout_clks <<= 1;
		timeout_us += (timeout_clks * 1000) /
			      (mmc_host_clk_rate(card->host) / 1000);

		erase_timeout = timeout_us / 1000;

		/*
		 * Theoretically, the calculation could underflow so round up
		 * to 1ms in that case.
		 */
		if (!erase_timeout)
			erase_timeout = 1;
	}

	/* Multiplier for secure operations */
	if (arg & MMC_SECURE_ARGS) {
		if (arg == MMC_SECURE_ERASE_ARG)
			erase_timeout *= card->ext_csd.sec_erase_mult;
		else
			erase_timeout *= card->ext_csd.sec_trim_mult;
	}

	erase_timeout *= qty;

	/*
	 * Ensure at least a 1 second timeout for SPI as per
	 * 'mmc_set_data_timeout()'
	 */
	if (mmc_host_is_spi(card->host) && erase_timeout < 1000)
		erase_timeout = 1000;

	return erase_timeout;
}

static unsigned int mmc_sd_erase_timeout(struct mmc_card *card,
					 unsigned int arg,
					 unsigned int qty)
{
	unsigned int erase_timeout;

	if (card->ssr.erase_timeout) {
		/* Erase timeout specified in SD Status Register (SSR) */
		erase_timeout = card->ssr.erase_timeout * qty +
				card->ssr.erase_offset;
	} else {
		/*
		 * Erase timeout not specified in SD Status Register (SSR) so
		 * use 250ms per write block.
		 */
		erase_timeout = 250 * qty;
	}

	/* Must not be less than 1 second */
	if (erase_timeout < 1000)
		erase_timeout = 1000;

	return erase_timeout;
}

static unsigned int mmc_erase_timeout(struct mmc_card *card,
				      unsigned int arg,
				      unsigned int qty)
{
	if (mmc_card_sd(card))
		return mmc_sd_erase_timeout(card, arg, qty);
	else
		return mmc_mmc_erase_timeout(card, arg, qty);
}

static int mmc_do_erase(struct mmc_card *card, unsigned int from,
			unsigned int to, unsigned int arg)
{
	struct mmc_command cmd = {0};
	unsigned int qty = 0;
	unsigned long timeout;
	unsigned int fr, nr;
	int err;

	fr = from;
	nr = to - from + 1;
	trace_mmc_blk_erase_start(arg, fr, nr);
	mmc_retune_hold(card->host);

	/*
	 * qty is used to calculate the erase timeout which depends on how many
	 * erase groups (or allocation units in SD terminology) are affected.
	 * We count erasing part of an erase group as one erase group.
	 * For SD, the allocation units are always a power of 2.  For MMC, the
	 * erase group size is almost certainly also power of 2, but it does not
	 * seem to insist on that in the JEDEC standard, so we fall back to
	 * division in that case.  SD may not specify an allocation unit size,
	 * in which case the timeout is based on the number of write blocks.
	 *
	 * Note that the timeout for secure trim 2 will only be correct if the
	 * number of erase groups specified is the same as the total of all
	 * preceding secure trim 1 commands.  Since the power may have been
	 * lost since the secure trim 1 commands occurred, it is generally
	 * impossible to calculate the secure trim 2 timeout correctly.
	 */
	if (card->erase_shift)
		qty += ((to >> card->erase_shift) -
			(from >> card->erase_shift)) + 1;
	else if (mmc_card_sd(card))
		qty += to - from + 1;
	else
		qty += ((to / card->erase_size) -
			(from / card->erase_size)) + 1;

	if (!mmc_card_blockaddr(card)) {
		from <<= 9;
		to <<= 9;
	}

	if (mmc_card_sd(card))
		cmd.opcode = SD_ERASE_WR_BLK_START;
	else
		cmd.opcode = MMC_ERASE_GROUP_START;
	cmd.arg = from;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_err("mmc_erase: group start error %d, "
		       "status %#x\n", err, cmd.resp[0]);
		err = -EIO;
		goto out;
	}

	memset(&cmd, 0, sizeof(struct mmc_command));
	if (mmc_card_sd(card))
		cmd.opcode = SD_ERASE_WR_BLK_END;
	else
		cmd.opcode = MMC_ERASE_GROUP_END;
	cmd.arg = to;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_err("mmc_erase: group end error %d, status %#x\n",
		       err, cmd.resp[0]);
		err = -EIO;
		goto out;
	}

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_ERASE;
	cmd.arg = arg;
	cmd.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	cmd.busy_timeout = mmc_erase_timeout(card, arg, qty);
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_err("mmc_erase: erase error %d, status %#x\n",
		       err, cmd.resp[0]);
		err = -EIO;
		goto out;
	}

	if (mmc_host_is_spi(card->host))
		goto out;

	timeout = jiffies + msecs_to_jiffies(MMC_CORE_TIMEOUT_MS);
	do {
		memset(&cmd, 0, sizeof(struct mmc_command));
		cmd.opcode = MMC_SEND_STATUS;
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
		/* Do not retry else we can't see errors */
		err = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (err || (cmd.resp[0] & 0xFDF92000)) {
			pr_err("error %d requesting status %#x\n",
				err, cmd.resp[0]);
			err = -EIO;
			goto out;
		}

		/* Timeout if the device never becomes ready for data and
		 * never leaves the program state.
		 */
		if (time_after(jiffies, timeout)) {
			pr_err("%s: Card stuck in programming state! %s\n",
				mmc_hostname(card->host), __func__);
			err =  -EIO;
			goto out;
		}

	} while (!(cmd.resp[0] & R1_READY_FOR_DATA) ||
		 (R1_CURRENT_STATE(cmd.resp[0]) == R1_STATE_PRG));
out:

	trace_mmc_blk_erase_end(arg, fr, nr);
	mmc_retune_release(card->host);
	return err;
}

/**
 * mmc_erase - erase sectors.
 * @card: card to erase
 * @from: first sector to erase
 * @nr: number of sectors to erase
 * @arg: erase command argument (SD supports only %MMC_ERASE_ARG)
 *
 * Caller must claim host before calling this function.
 */
int mmc_erase(struct mmc_card *card, unsigned int from, unsigned int nr,
	      unsigned int arg)
{
	unsigned int rem, to = from + nr;

	if (!(card->host->caps & MMC_CAP_ERASE) ||
	    !(card->csd.cmdclass & CCC_ERASE))
		return -EOPNOTSUPP;

	if (!card->erase_size)
		return -EOPNOTSUPP;

	if (mmc_card_sd(card) && arg != MMC_ERASE_ARG)
		return -EOPNOTSUPP;

	if ((arg & MMC_SECURE_ARGS) &&
	    !(card->ext_csd.sec_feature_support & EXT_CSD_SEC_ER_EN))
		return -EOPNOTSUPP;

	if ((arg & MMC_TRIM_ARGS) &&
	    !(card->ext_csd.sec_feature_support & EXT_CSD_SEC_GB_CL_EN))
		return -EOPNOTSUPP;

	if (arg == MMC_SECURE_ERASE_ARG) {
		if (from % card->erase_size || nr % card->erase_size)
			return -EINVAL;
	}

	if (arg == MMC_ERASE_ARG) {
		rem = from % card->erase_size;
		if (rem) {
			rem = card->erase_size - rem;
			from += rem;
			if (nr > rem)
				nr -= rem;
			else
				return 0;
		}
		rem = nr % card->erase_size;
		if (rem)
			nr -= rem;
	}

	if (nr == 0)
		return 0;

	to = from + nr;

	if (to <= from)
		return -EINVAL;

	/* 'from' and 'to' are inclusive */
	to -= 1;

	return mmc_do_erase(card, from, to, arg);
}
EXPORT_SYMBOL(mmc_erase);

int mmc_can_erase(struct mmc_card *card)
{
	if ((card->host->caps & MMC_CAP_ERASE) &&
	    (card->csd.cmdclass & CCC_ERASE) && card->erase_size)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_erase);

int mmc_can_trim(struct mmc_card *card)
{
	if (card->ext_csd.sec_feature_support & EXT_CSD_SEC_GB_CL_EN)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_trim);

int mmc_can_discard(struct mmc_card *card)
{
	/*
	 * As there's no way to detect the discard support bit at v4.5
	 * use the s/w feature support filed.
	 */
	if (card->ext_csd.feature_support & MMC_DISCARD_FEATURE)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_discard);

int mmc_can_sanitize(struct mmc_card *card)
{
	if (!mmc_can_trim(card) && !mmc_can_erase(card))
		return 0;
	if (card->ext_csd.sec_feature_support & EXT_CSD_SEC_SANITIZE)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_sanitize);

int mmc_can_secure_erase_trim(struct mmc_card *card)
{
	if ((card->ext_csd.sec_feature_support & EXT_CSD_SEC_ER_EN) &&
	    !(card->quirks & MMC_QUIRK_SEC_ERASE_TRIM_BROKEN))
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_secure_erase_trim);

int mmc_erase_group_aligned(struct mmc_card *card, unsigned int from,
			    unsigned int nr)
{
	if (!card->erase_size)
		return 0;
	if (from % card->erase_size || nr % card->erase_size)
		return 0;
	return 1;
}
EXPORT_SYMBOL(mmc_erase_group_aligned);

static unsigned int mmc_do_calc_max_discard(struct mmc_card *card,
					    unsigned int arg)
{
	struct mmc_host *host = card->host;
	unsigned int max_discard, x, y, qty = 0, max_qty, timeout;
	unsigned int last_timeout = 0;

	if (card->erase_shift)
		max_qty = UINT_MAX >> card->erase_shift;
	else if (mmc_card_sd(card))
		max_qty = UINT_MAX;
	else
		max_qty = UINT_MAX / card->erase_size;

	/* Find the largest qty with an OK timeout */
	do {
		y = 0;
		for (x = 1; x && x <= max_qty && max_qty - x >= qty; x <<= 1) {
			timeout = mmc_erase_timeout(card, arg, qty + x);
			if (timeout > host->max_busy_timeout)
				break;
			if (timeout < last_timeout)
				break;
			last_timeout = timeout;
			y = x;
		}
		qty += y;
	} while (y);

	if (!qty)
		return 0;

	if (qty == 1)
		return 1;

	/* Convert qty to sectors */
	if (card->erase_shift)
		max_discard = --qty << card->erase_shift;
	else if (mmc_card_sd(card))
		max_discard = qty;
	else
		max_discard = --qty * card->erase_size;

	return max_discard;
}

unsigned int mmc_calc_max_discard(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	unsigned int max_discard, max_trim;

	if (!host->max_busy_timeout)
		return UINT_MAX;

	/*
	 * Without erase_group_def set, MMC erase timeout depends on clock
	 * frequence which can change.  In that case, the best choice is
	 * just the preferred erase size.
	 */
	if (mmc_card_mmc(card) && !(card->ext_csd.erase_group_def & 1))
		return card->pref_erase;

	max_discard = mmc_do_calc_max_discard(card, MMC_ERASE_ARG);
	if (mmc_can_trim(card)) {
		max_trim = mmc_do_calc_max_discard(card, MMC_TRIM_ARG);
		if (max_trim < max_discard)
			max_discard = max_trim;
	} else if (max_discard < card->erase_size) {
		max_discard = 0;
	}
	pr_debug("%s: calculated max. discard sectors %u for timeout %u ms\n",
		 mmc_hostname(host), max_discard, host->max_busy_timeout);
	return max_discard;
}
EXPORT_SYMBOL(mmc_calc_max_discard);

int mmc_set_blocklen(struct mmc_card *card, unsigned int blocklen)
{
	struct mmc_command cmd = {0};

	if (mmc_card_blockaddr(card) || mmc_card_ddr52(card))
		return 0;

	cmd.opcode = MMC_SET_BLOCKLEN;
	cmd.arg = blocklen;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	return mmc_wait_for_cmd(card->host, &cmd, 5);
}
EXPORT_SYMBOL(mmc_set_blocklen);

int mmc_set_blockcount(struct mmc_card *card, unsigned int blockcount,
			bool is_rel_write)
{
	struct mmc_command cmd = {0};

	cmd.opcode = MMC_SET_BLOCK_COUNT;
	cmd.arg = blockcount & 0x0000FFFF;
	if (is_rel_write)
		cmd.arg |= 1 << 31;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	return mmc_wait_for_cmd(card->host, &cmd, 5);
}
EXPORT_SYMBOL(mmc_set_blockcount);

static void mmc_hw_reset_for_init(struct mmc_host *host)
{
	if (!(host->caps & MMC_CAP_HW_RESET) || !host->ops->hw_reset)
		return;
	mmc_host_clk_hold(host);
	host->ops->hw_reset(host);
	mmc_host_clk_release(host);
}

int mmc_hw_reset(struct mmc_host *host)
{
	int ret;

	if (!host->card)
		return -EINVAL;

	mmc_bus_get(host);
	if (!host->bus_ops || host->bus_dead || !host->bus_ops->reset) {
		mmc_bus_put(host);
		return -EOPNOTSUPP;
	}

	ret = host->bus_ops->reset(host);
	mmc_bus_put(host);

	if (ret != -EOPNOTSUPP)
		pr_warn("%s: tried to reset card\n", mmc_hostname(host));

	return ret;
}
EXPORT_SYMBOL(mmc_hw_reset);

static int mmc_rescan_try_freq(struct mmc_host *host, unsigned freq)
{
	host->f_init = freq;

#ifdef CONFIG_MMC_DEBUG
	pr_info("%s: %s: trying to init card at %u Hz\n",
		mmc_hostname(host), __func__, host->f_init);
#endif
	mmc_power_up(host, host->ocr_avail);

	/*
	 * Some eMMCs (with VCCQ always on) may not be reset after power up, so
	 * do a hardware reset if possible.
	 */
	mmc_hw_reset_for_init(host);

	/*
	 * sdio_reset sends CMD52 to reset card.  Since we do not know
	 * if the card is being re-initialized, just send it.  CMD52
	 * should be ignored by SD/eMMC cards.
	 */
	sdio_reset(host);
	mmc_go_idle(host);

	mmc_send_if_cond(host, host->ocr_avail);

	/* Order's important: probe SDIO, then SD, then MMC */
	if (!mmc_attach_sdio(host))
		return 0;
	if (!mmc_attach_sd(host))
		return 0;
	if (!mmc_attach_mmc(host))
		return 0;

	mmc_power_off(host);
	return -EIO;
}

int _mmc_detect_card_removed(struct mmc_host *host)
{
	int ret;

	if (host->caps & MMC_CAP_NONREMOVABLE)
		return 0;

	if (!host->card || mmc_card_removed(host->card))
		return 1;

	ret = host->bus_ops->alive(host);

	/*
	 * Card detect status and alive check may be out of sync if card is
	 * removed slowly, when card detect switch changes while card/slot
	 * pads are still contacted in hardware (refer to "SD Card Mechanical
	 * Addendum, Appendix C: Card Detection Switch"). So reschedule a
	 * detect work 200ms later for this case.
	 */
	if (!ret && host->ops->get_cd && !host->ops->get_cd(host)) {
		mmc_detect_change(host, msecs_to_jiffies(200));
		pr_debug("%s: card removed too slowly\n", mmc_hostname(host));
	}

	if (ret) {
		mmc_card_set_removed(host->card);
		pr_debug("%s: card remove detected\n", mmc_hostname(host));
	}

	return ret;
}

int mmc_detect_card_removed(struct mmc_host *host)
{
	struct mmc_card *card = host->card;
	int ret;

	WARN_ON(!host->claimed);

	if (!card)
		return 1;

	ret = mmc_card_removed(card);
	/*
	 * The card will be considered unchanged unless we have been asked to
	 * detect a change or host requires polling to provide card detection.
	 */
	if (!host->detect_change && !(host->caps & MMC_CAP_NEEDS_POLL))
		return ret;

	host->detect_change = 0;
	if (!ret) {
		ret = _mmc_detect_card_removed(host);
		if (ret && (host->caps & MMC_CAP_NEEDS_POLL)) {
			/*
			 * Schedule a detect work as soon as possible to let a
			 * rescan handle the card removal.
			 */
			cancel_delayed_work(&host->detect);
			_mmc_detect_change(host, 0, false);
		}
	}

	return ret;
}
EXPORT_SYMBOL(mmc_detect_card_removed);

void mmc_rescan(struct work_struct *work)
{
	struct mmc_host *host =
		container_of(work, struct mmc_host, detect.work);
	int i;

	if (host->trigger_card_event && host->ops->card_event) {
		host->ops->card_event(host);
		host->trigger_card_event = false;
	}

	if (host->rescan_disable)
		return;

	/* If there is a non-removable card registered, only scan once */
	if ((host->caps & MMC_CAP_NONREMOVABLE) && host->rescan_entered)
		return;
	host->rescan_entered = 1;

	mmc_bus_get(host);

	/*
	 * if there is a _removable_ card registered, check whether it is
	 * still present
	 */
	if (host->bus_ops && !host->bus_dead
	    && !(host->caps & MMC_CAP_NONREMOVABLE))
		host->bus_ops->detect(host);

	host->detect_change = 0;

	/*
	 * Let mmc_bus_put() free the bus/bus_ops if we've found that
	 * the card is no longer present.
	 */
	mmc_bus_put(host);
	mmc_bus_get(host);

	/* if there still is a card present, stop here */
	if (host->bus_ops != NULL) {
		mmc_bus_put(host);
		goto out;
	}

	/*
	 * Only we can add a new handler, so it's safe to
	 * release the lock here.
	 */
	mmc_bus_put(host);

	if (!(host->caps & MMC_CAP_NONREMOVABLE) && host->ops->get_cd &&
			host->ops->get_cd(host) == 0) {
		mmc_claim_host(host);
		mmc_power_off(host);
		mmc_release_host(host);
		goto out;
	}

	mmc_claim_host(host);
	for (i = 0; i < ARRAY_SIZE(freqs); i++) {
		if (!mmc_rescan_try_freq(host, max(freqs[i], host->f_min)))
		{
			host->caps &= ~MMC_CAP_NEEDS_POLL;
			pr_debug("%s (polling disabled)", mmc_hostname(host));
			break;
		}
		if (freqs[i] <= host->f_min)
			break;
	}
	mmc_release_host(host);

 out:
	if (host->caps & MMC_CAP_NEEDS_POLL)
		mmc_schedule_delayed_work(&host->detect, HZ);
	/*
	 * If retry_timeout do not define in advance, set as 10
	 */
	if(host->retry_timeout > 10)
		host->retry_timeout = 10;

	host->retry_timeout--;
	if (host->retry_timeout == 0)
		host->caps &= ~MMC_CAP_NEEDS_POLL;
	else
		pr_debug("%s: reschedule detect work: %d\n",

	mmc_hostname(host), host->retry_timeout);
}

void mmc_start_host(struct mmc_host *host)
{
	host->f_init = max(freqs[0], host->f_min);
	host->rescan_disable = 0;
	host->ios.power_mode = MMC_POWER_UNDEFINED;
	if (host->caps2 & MMC_CAP2_NO_PRESCAN_POWERUP)
		mmc_power_off(host);
	else
		mmc_power_up(host, host->ocr_avail);
	mmc_gpiod_request_cd_irq(host);
	_mmc_detect_change(host, 0, false);
}

void mmc_stop_host(struct mmc_host *host)
{
#ifdef CONFIG_MMC_DEBUG
	unsigned long flags;
	spin_lock_irqsave(&host->lock, flags);
	host->removed = 1;
	spin_unlock_irqrestore(&host->lock, flags);
#endif
	if (host->slot.cd_irq >= 0)
		disable_irq(host->slot.cd_irq);

	host->rescan_disable = 1;
	cancel_delayed_work_sync(&host->detect);
	mmc_flush_scheduled_work();

	/* clear pm flags now and let card drivers set them as needed */
	host->pm_flags = 0;

	mmc_bus_get(host);
	if (host->bus_ops && !host->bus_dead) {
		/* Calling bus_ops->remove() with a claimed host can deadlock */
		host->bus_ops->remove(host);
		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
		mmc_bus_put(host);
		return;
	}
	mmc_bus_put(host);

	BUG_ON(host->card);

	mmc_power_off(host);
}

int mmc_power_save_host(struct mmc_host *host)
{
	int ret = 0;

#ifdef CONFIG_MMC_DEBUG
	pr_info("%s: %s: powering down\n", mmc_hostname(host), __func__);
#endif

	mmc_bus_get(host);

	if (!host->bus_ops || host->bus_dead) {
		mmc_bus_put(host);
		return -EINVAL;
	}

	if (host->bus_ops->power_save)
		ret = host->bus_ops->power_save(host);

	mmc_bus_put(host);

	mmc_power_off(host);

	return ret;
}
EXPORT_SYMBOL(mmc_power_save_host);

int mmc_power_restore_host(struct mmc_host *host)
{
	int ret;

#ifdef CONFIG_MMC_DEBUG
	pr_info("%s: %s: powering up\n", mmc_hostname(host), __func__);
#endif

	mmc_bus_get(host);

	if (!host->bus_ops || host->bus_dead) {
		mmc_bus_put(host);
		return -EINVAL;
	}

	mmc_power_up(host, host->card->ocr);
	ret = host->bus_ops->power_restore(host);

	mmc_bus_put(host);

	return ret;
}
EXPORT_SYMBOL(mmc_power_restore_host);

/*
 * Flush the cache to the non-volatile storage.
 */
int mmc_flush_cache(struct mmc_card *card)
{
	int err = 0;

	if (mmc_card_mmc(card) &&
			(card->ext_csd.cache_size > 0) &&
			(card->ext_csd.cache_ctrl & 1)) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_FLUSH_CACHE, 1, 0);
		if (err)
			pr_err("%s: cache flush error %d\n",
					mmc_hostname(card->host), err);
	}

	return err;
}
EXPORT_SYMBOL(mmc_flush_cache);

#ifdef CONFIG_PM

/* Do the card removal on suspend if card is assumed removeable
 * Do that in pm notifier while userspace isn't yet frozen, so we will be able
   to sync the card.
*/
int mmc_pm_notify(struct notifier_block *notify_block,
					unsigned long mode, void *unused)
{
	struct mmc_host *host = container_of(
		notify_block, struct mmc_host, pm_notify);
	unsigned long flags;
	int err = 0;

	switch (mode) {
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
	case PM_RESTORE_PREPARE:
#ifdef MTK_BKOPS_IDLE_MAYA
		if (host->card && mmc_card_mmc(host->card)) {
			mmc_claim_host(host);
			err = mmc_stop_bkops(host->card);
			mmc_release_host(host);
			if (err) {
				pr_err("%s: didn't stop bkops\n", mmc_hostname(host));
				return err;
			}
		}
#endif
		spin_lock_irqsave(&host->lock, flags);
		host->rescan_disable = 1;
		spin_unlock_irqrestore(&host->lock, flags);
		cancel_delayed_work_sync(&host->detect);

		if (!host->bus_ops)
			break;

		/* Validate prerequisites for suspend */
		if (host->bus_ops->pre_suspend)
			err = host->bus_ops->pre_suspend(host);
		if (!err)
			break;

		/* Calling bus_ops->remove() with a claimed host can deadlock */
		host->bus_ops->remove(host);
		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
		host->pm_flags = 0;
		break;

	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:

		spin_lock_irqsave(&host->lock, flags);
		host->rescan_disable = 0;
		spin_unlock_irqrestore(&host->lock, flags);
		_mmc_detect_change(host, 0, false);

	}

	return 0;
}
#endif

/**
 * mmc_init_context_info() - init synchronization context
 * @host: mmc host
 *
 * Init struct context_info needed to implement asynchronous
 * request mechanism, used by mmc core, host driver and mmc requests
 * supplier.
 */
void mmc_init_context_info(struct mmc_host *host)
{
	spin_lock_init(&host->context_info.lock);
	host->context_info.is_new_req = false;
	host->context_info.is_done_rcv = false;
	host->context_info.is_waiting_last_req = false;
	init_waitqueue_head(&host->context_info.wait);
}

#ifdef CONFIG_MMC_EMBEDDED_SDIO
void mmc_set_embedded_sdio_data(struct mmc_host *host,
				struct sdio_cis *cis,
				struct sdio_cccr *cccr,
				struct sdio_embedded_func *funcs,
				int num_funcs)
{
	host->embedded_sdio_data.cis = cis;
	host->embedded_sdio_data.cccr = cccr;
	host->embedded_sdio_data.funcs = funcs;
	host->embedded_sdio_data.num_funcs = num_funcs;
}

EXPORT_SYMBOL(mmc_set_embedded_sdio_data);
#endif

static int __init mmc_init(void)
{
	int ret;

	workqueue = alloc_ordered_workqueue("kmmcd", 0);
	if (!workqueue)
		return -ENOMEM;

	ret = mmc_register_bus();
	if (ret)
		goto destroy_workqueue;

	ret = mmc_register_host_class();
	if (ret)
		goto unregister_bus;

	ret = sdio_register_bus();
	if (ret)
		goto unregister_host_class;

	return 0;

unregister_host_class:
	mmc_unregister_host_class();
unregister_bus:
	mmc_unregister_bus();
destroy_workqueue:
	destroy_workqueue(workqueue);

	return ret;
}

static void __exit mmc_exit(void)
{
	sdio_unregister_bus();
	mmc_unregister_host_class();
	mmc_unregister_bus();
	destroy_workqueue(workqueue);
}

subsys_initcall(mmc_init);
module_exit(mmc_exit);

MODULE_LICENSE("GPL");
