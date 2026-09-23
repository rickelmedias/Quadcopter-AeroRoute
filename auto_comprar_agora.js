(() => {
  const MIN_INTERVAL_MS = 5000;
  const MAX_INTERVAL_MS = 12000;
  const STORAGE_KEY = 'autoComprarAgoraRunning';

  const findButton = () =>
    document.querySelector('button[aria-label="Comprar agora"]') ||
    [...document.querySelectorAll('button')].find((button) =>
      button.textContent.trim().toLowerCase().includes('comprar agora')
    );

  const check = () => {
    const button = findButton();
    const disabled = button?.getAttribute('aria-disabled') === 'true' ||
      button?.disabled === true;

    if (button && !disabled) {
      sessionStorage.removeItem(STORAGE_KEY);
      button.click();
      console.log('[auto-comprar] Botão "Comprar agora" encontrado e clicado.');
      return;
    }

    sessionStorage.setItem(STORAGE_KEY, '1');
    const interval = Math.floor(
      Math.random() * (MAX_INTERVAL_MS - MIN_INTERVAL_MS + 1) + MIN_INTERVAL_MS
    );
    console.log(`[auto-comprar] Botão indisponível; recarregando em ${Math.round(interval / 1000)} segundos.`);
    setTimeout(() => location.reload(), interval);
  };

  check();
})();
