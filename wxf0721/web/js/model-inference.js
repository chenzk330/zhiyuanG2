// model-inference.js
// 模型推理：模型列表 + 悬浮推理按钮

export default {
    name: 'ModelInference',
    template: `
    <div class="panel">
        <h5>模型推理</h5>

        <table class="dc-table">
            <thead>
                <tr>
                    <th>模型名称</th>
                    <th>模型类型</th>
                    <th>需要内存</th>
                    <th>需要显存</th>
                    <th>模式</th>
                </tr>
            </thead>
            <tbody>
                <tr v-for="(m, i) in models" :key="i"
                    @click="selectedModel = i"
                    :class="{ selected: selectedModel === i }">
                    <td>{{ m.name }}</td>
                    <td>{{ m.type }}</td>
                    <td>{{ m.mem }}</td>
                    <td>{{ m.vram }}</td>
                    <td>{{ m.mode }}</td>
                </tr>
            </tbody>
        </table>

        <!-- 悬浮推理按钮 -->
        <button v-if="selectedModel >= 0"
                class="floating-btn run"
                @click="doInference">
            推理
        </button>
    </div>
    `,
    data() {
        return {
            selectedModel: -1,
            models: [
                { name: 'act_grab_v1',    type: 'act',   mem: '4GB',  vram: '2GB',  mode: '双臂' },
                { name: 'dp_place_v2',    type: 'dp',    mem: '8GB',  vram: '6GB',  mode: '单臂' },
                { name: 'pi05_full_v1',   type: 'pi0.5', mem: '16GB', vram: '12GB', mode: '全身' },
                { name: 'act_pull_v3',    type: 'act',   mem: '4GB',  vram: '2GB',  mode: '单臂' },
                { name: 'dp_assembly_v1', type: 'dp',    mem: '8GB',  vram: '4GB',  mode: '双臂' },
            ]
        };
    },
    methods: {
        doInference() {
            const m = this.models[this.selectedModel];
            alert(`开始推理: ${m.name} (${m.type})`);
        }
    }
};
