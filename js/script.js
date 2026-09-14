// Flip project cards on tap for touch devices (hover already handles desktop via CSS).
document.querySelectorAll(".project-card").forEach((card) => {
  card.addEventListener("click", () => {
    card.classList.toggle("is-flipped");
  });
});
