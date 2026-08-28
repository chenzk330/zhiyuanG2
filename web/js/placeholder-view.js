// placeholder-view.js
// 占位组件：用于尚未实现功能的菜单项

export default {
    name: 'PlaceholderView',
    props: {
        title: { type: String, default: '' }
    },
    template: `
    <div class="panel">
        <h5>{{ title }}</h5>
        <div class="placeholder-msg">
            <div class="placeholder-icon">🔧</div>
            <div class="placeholder-text">功能开发中...</div>
        </div>
    </div>
    `
};
