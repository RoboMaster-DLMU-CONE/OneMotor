#ifndef ONEMOTOR_PIDCHAIN_HPP
#define ONEMOTOR_PIDCHAIN_HPP
#include <tuple>
#include "PID.hpp"

namespace OneMotor::Control
{
    /**
     * @brief PID处理链。
     * @tparam Node 当前节点的PID控制器类型
     * @tparam RestNodes 剩余节点的类型
     */
    template <typename Node, typename... RestNodes>
    class PIDChain
    {
    public:
        static constexpr size_t Size = 1 + sizeof...(RestNodes);
        constexpr size_t size() { return Size; }

        // 构造函数：递归初始化
        template <typename Param, typename... RestParams>
        explicit PIDChain(const Param& p, const RestParams&... rest)
            : head_node(p), tail(rest...)
        {
        }

        /**
         * @brief 递归计算逻辑
         * @param target 最终目标值（对于第一级）或上一级的输出（对于后续级）
         * @param measure 当前级的测量值
         * @param rest_measures 剩余层级的测量值
         */
        template <typename T, typename... Ms>
        auto compute(T target, T measure, Ms... rest_measures)
        {
            // 1. 计算当前级的输出
            auto current_output = head_node.compute(target, measure);

            // 2. 将当前输出作为下一级的 target，递归传递剩余测量值
            // 如果这是最后一级，tail.compute 会处理
            return tail.compute(current_output, rest_measures...);
        }

        // 重置所有状态
        void reset()
        {
            head_node.reset();
            tail.reset();
        }

        // 获取特定层级的引用（用于调试或运行时参数修改）
        template <size_t Index>
        auto& get()
        {
            if constexpr (Index == 0) return head_node;
            else return tail.template get<Index - 1>();
        }

    private:
        Node head_node; ///< 当前级的PID控制器
        PIDChain<RestNodes...> tail; ///< 剩余的级联部分
    };

    /**
     * @brief 特化：链中最后一个节点。
     * 这是递归的终点。
     */
    template <typename Node>
    class PIDChain<Node>
    {
    public:
        template <typename Param>
        explicit PIDChain(const Param& p) : head_node(p)
        {
        }

        template <typename T>
        auto compute(T target, T measure)
        {
            return head_node.compute(target, measure);
        }

        void reset() { head_node.reset(); }

        template <size_t Index>
        auto& get()
        {
            static_assert(Index == 0, "Index out of bounds");
            return head_node;
        }

    private:
        Node head_node;
    };

    template <typename... Nodes>
    class ChainBuilder;

    // 空构建器
    template <>
    class ChainBuilder<>
    {
    public:
        // 添加一级 PID
        // TParams 是 PID_Params<...>
        // Features 是 PIDController 的特性标签
        template <typename Algorithm = Positional, typename ValueType = float, typename... Features>
        auto add(const PID_Params<ValueType>& params)
        {
            // 定义新节点的类型
            using NewNode = PIDController<Algorithm, ValueType, Features...>;

            // 返回包含新类型的新构建器，并传递参数
            return ChainBuilder<NewNode>(std::make_tuple(params));
        }
    };

    // 有状态的构建器
    template <typename... ExistingNodes>
    class ChainBuilder
    {
        using ParamsTuple = std::tuple<typename ExistingNodes::ParamsType...>;
        ParamsTuple params_store;

    public:
        explicit ChainBuilder(ParamsTuple p) : params_store(std::move(p))
        {
        }

        template <typename Algorithm = Positional, typename ValueType = float, typename... Features>
        auto add(const PID_Params<ValueType>& new_params)
        {
            using NewNode = PIDController<Algorithm, ValueType, Features...>;

            // 合并旧参数和新参数
            auto new_params_tuple = std::tuple_cat(params_store, std::make_tuple(new_params));

            return ChainBuilder<ExistingNodes..., NewNode>(new_params_tuple);
        }

        // 构建最终的 PIDChain 对象
        auto build()
        {
            return std::apply([]<typename... T>(T&&... args)
            {
                return PIDChain<ExistingNodes...>(std::forward<T>(args)...);
            }, params_store);
        }
    };

    // 辅助函数，创建构建器
    inline auto createPIDChain()
    {
        return ChainBuilder<>{};
    }
}

#endif //ONEMOTOR_PIDCHAIN_HPP
