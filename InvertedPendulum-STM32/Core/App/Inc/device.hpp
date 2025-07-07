#ifndef DEVICE__HPP_
#define DEVICE__HPP_

class IDevice {
 public:
  virtual void initialize() = 0;  // デバイスの初期化
};

#endif /* DEVICE__HPP_ */
