import { defineConfig } from 'vite';

export default defineConfig({
  root: '.',
  build: {
    outDir: 'dist',
    emptyOutDir: true,
    rollupOptions: {
      input: {
        main: 'index.html',
        editor: 'editor.html',
        'truck-physics-demo': 'truck-physics-demo.html'
      }
    }
  }
});
