(() => {
  const MIN_INTERVAL_MS = 5000;
  const MAX_INTERVAL_MS = 12000;
  const STORAGE_KEY = 'autoAddToCartRunning';

  const findButton = () =>
    document.querySelector('button[aria-label="Adicionar ao carrinho"]') ||
    [...document.querySelectorAll('button')].find((button) =>
      button.textContent.trim().toLowerCase().includes('adicionar ao carrinho')
    );

  const stop = () => {
    sessionStorage.removeItem(STORAGE_KEY);
  };

  const check = () => {
    const button = findButton();
    const disabled = button?.getAttribute('aria-disabled') === 'true' ||
      button?.disabled === true;

    if (button && !disabled) {
      stop();
      button.click();
      console.log('[auto-add] Botão encontrado e clicado. Atualização interrompida.');
      return;
    }

    sessionStorage.setItem(STORAGE_KEY, '1');
    const interval = Math.floor(
      Math.random() * (MAX_INTERVAL_MS - MIN_INTERVAL_MS + 1) + MIN_INTERVAL_MS
    );
    console.log(`[auto-add] Botão ainda não disponível; recarregando em ${Math.round(interval / 1000)} segundos.`);
    setTimeout(() => location.reload(), interval);
  };

  // Evita iniciar mais de uma instância caso o script seja executado novamente.
  if (sessionStorage.getItem(STORAGE_KEY) !== '1') {
    sessionStorage.setItem(STORAGE_KEY, '1');
  }

  check();
})();
