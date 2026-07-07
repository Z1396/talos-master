// 订阅系统基类、回调上下文定义头文件
#include "scheduler/rclcompat/system.hpp"

namespace talos::scheduler::rclcompat {

/**
 * @brief 线程本地存储全局回调上下文单例
 * thread_local 修饰：每个工作线程拥有独立实例，线程间完全隔离，无锁安全
 * 作用：订阅回调执行时标记当前正在运行的RclSubSystemBase系统指针
 */
thread_local CallbackContext g_callback_context;

/**
 * @brief RAII 作用域守卫，自动切换/恢复线程本地回调上下文
 * 构造时绑定当前订阅系统到线程局部g_callback_context
 * 析构时自动还原上一次保存的系统指针，避免上下文污染
 * @param current 当前执行回调的订阅系统指针 RclSubSystemBase
 */
CallbackContextScope::CallbackContextScope(RclSubSystemBase* current) noexcept
    // 绑定当前线程独有的上下文实例
    : ctx_(&g_callback_context)
    // 保存切换前旧系统指针，析构用于恢复
    , prev_system_(ctx_->current_system) {
    // 覆盖上下文当前系统为本次待执行的订阅系统
    ctx_->current_system = current;
}

/**
 * @brief 析构函数：RAII自动回滚上下文
 * 离开当前回调作用域，把线程本地上下文恢复为进入作用域前的值
 * 防止下一次回调拿到错误的系统指针
 */
CallbackContextScope::~CallbackContextScope() noexcept { ctx_->current_system = prev_system_; }

/**
 * @brief 获取订阅系统静态元数据（通道、调度策略、系统名等）
 * @return 只读引用 SystemMeta 系统元信息
 */
const SystemMeta& RclSubSystemBase::meta() const noexcept { return meta_; }

} // namespace talos::scheduler::rclcompat