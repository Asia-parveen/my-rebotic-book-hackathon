import React from 'react';
import ReactDOM from 'react-dom/client';
import ChatbotWidget from '@site/src/components/ChatbotWidget';

// Create a React root and render the chatbot widget
function renderChatbot() {
  const container = document.getElementById('chatbot-root');
  if (container) {
    const root = ReactDOM.createRoot(container);
    root.render(<ChatbotWidget />);
  }
}

// Wait for the DOM to be fully loaded before rendering
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', renderChatbot);
} else {
  renderChatbot();
}