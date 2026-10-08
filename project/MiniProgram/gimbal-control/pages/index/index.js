const DEVICE_URL = 'http://192.168.4.1'

const MODE_LABELS = {
  horizontal: '左右扫描',
  vertical: '上下扫描',
  all: '全范围扫描',
  stop: '已停止'
}

Page({
  data: {
    online: false,
    busy: false,
    pendingMode: '',
    modeLabel: '等待连接',
    roll: 0,
    pitch: 0,
    distanceText: '--'
  },

  onLoad() {
    this.updateStatus()
    this.statusTimer = setInterval(() => this.updateStatus(), 1000)
  },

  onUnload() {
    if (this.statusTimer) clearInterval(this.statusTimer)
  },

  startScan(event) {
    const mode = event.currentTarget.dataset.mode
    this.setData({ busy: true, pendingMode: mode })

    wx.request({
      url: `${DEVICE_URL}/api/scan?mode=${mode}`,
      method: 'GET',
      timeout: 3000,
      success: (response) => {
        if (response.statusCode === 200 && response.data.ok) {
          this.setData({ modeLabel: MODE_LABELS[mode] })
          wx.showToast({ title: response.data.message, icon: 'none' })
        } else {
          wx.showToast({ title: '指令未被设备接受', icon: 'none' })
        }
      },
      fail: () => {
        this.setData({ online: false })
        wx.showToast({ title: '请连接 Gimbal-8266', icon: 'none' })
      },
      complete: () => this.setData({ busy: false, pendingMode: '' })
    })
  },

  updateStatus() {
    wx.request({
      url: `${DEVICE_URL}/api/status`,
      method: 'GET',
      timeout: 1500,
      success: (response) => {
        const status = response.data
        this.setData({
          online: Boolean(status.online),
          roll: Number(status.roll || 0),
          pitch: Number(status.pitch || 0),
          distanceText: Number(status.distance) < 0 ? '--' : `${status.distance} cm`
        })
      },
      fail: () => this.setData({ online: false })
    })
  }
})
