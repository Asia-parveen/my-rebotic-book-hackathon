// This will only run in the browser, not during SSR
// The plugin system ensures this only executes client-side
if (typeof window !== 'undefined') {
  // Wait for DOM to be ready and then dynamically load and render the chatbot
  window.addEventListener('load', function() {
    // Dynamically import and render the chatbot
    import('@site/src/components/ChatbotWidget').then(module => {
      const ChatbotWidget = module.default;
      if (typeof document !== 'undefined') {
        const container = document.getElementById('chatbot-root');
        if (container) {
          import('react-dom/client').then(ReactDOMClient => {
            const root = ReactDOMClient.createRoot(container);
            import('react').then(ReactModule => {
              root.render(ReactModule.createElement(ChatbotWidget));
            });
          });
        }
      }
    });
  });
}