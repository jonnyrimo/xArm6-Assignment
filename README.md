# Your Name — Portfolio

A simple 3-page portfolio site (Home, Career, Projects) built with plain HTML/CSS/JS —
no build tools, no frameworks. Ready to publish with GitHub Pages.

## File structure

```
portfolio/
├── index.html          Home page
├── career.html         Career & academic history
├── projects.html       Projects grid with flip cards
├── css/
│   └── style.css       All styling
├── js/
│   └── script.js       Makes project cards flip on tap (mobile)
└── images/
    ├── hero-background.jpg      Home page background (placeholder — replace me)
    └── projects/
        ├── project-1.jpg        Placeholder — replace with your project photos
        ├── project-2.jpg
        └── project-3.jpg
```

## 1. Customize the content

Everything is plain text in the HTML files — open them in any text editor (VS Code, etc.).

**index.html**
- Replace "Your Name", "Your role or field", the hero heading/paragraph, the About text,
  and the items inside `<ul class="skills-list">` and `<ul class="interests">`.

**career.html**
- Each job or degree is one `<div class="timeline-item">` block. Edit the dates, title,
  organization, and description. Copy/paste a block to add more entries — newest first
  reads best, but order them however you like.

**projects.html**
- Each project is one `<button class="project-card">` block. Edit the `<img src="...">`
  path, the `alt` text, the `<h3>` title, and the description `<p>`. Copy/paste the whole
  block to add more projects — the grid will automatically wrap to new rows.

**Sidebar (all three pages)**
- The sidebar is repeated at the top of each HTML file. Update your name, tagline, and the
  email/GitHub/LinkedIn links (all three files need the same edits, since there's no shared
  template in plain HTML).

## 2. Add your own images

- Replace `images/hero-background.jpg` with any wide photo or graphic you'd like as the
  home page background. Keep the same filename, or update the path in `index.html`:
  ```html
  <section class="hero" style="--hero-image: url('images/hero-background.jpg');">
  ```
- Replace the files in `images/projects/` with your real project photos. They look best
  as square-ish images (the cards crop to a square automatically). You can use any
  filenames you like as long as you update the `src` in `projects.html` to match.
- To add a project photo that doesn't exist yet, just drop the image file into
  `images/projects/` and reference it from a new `.project-card` block.

## 3. Preview it locally (optional but recommended)

You can just double-click `index.html` to open it in a browser. For the most accurate
preview (some browsers restrict local file loading), run a tiny local server instead:

```bash
cd portfolio
python3 -m http.server 8000
```

Then open `http://localhost:8000` in your browser.

## 4. Publish with GitHub Pages

1. **Create a repository.** On GitHub, click "New repository." Name it whatever you like
   (e.g. `portfolio`), set it to Public, and create it without a README (you already have
   files).
2. **Push your files.** From inside the `portfolio` folder on your computer:
   ```bash
   git init
   git add .
   git commit -m "Initial portfolio site"
   git branch -M main
   git remote add origin https://github.com/YOUR-USERNAME/YOUR-REPO.git
   git push -u origin main
   ```
3. **Enable Pages.** On GitHub, go to your repo → **Settings** → **Pages** (left sidebar).
   Under "Build and deployment," set **Source** to "Deploy from a branch," pick the
   **main** branch and the **/ (root)** folder, then click **Save**.
4. **Wait a minute**, then refresh that Pages settings screen — it will show your live
   URL, typically:
   ```
   https://YOUR-USERNAME.github.io/YOUR-REPO/
   ```
5. **Updating the site later:** just edit the files, then run
   ```bash
   git add .
   git commit -m "Update content"
   git push
   ```
   GitHub Pages redeploys automatically within a minute or two of every push.

### Optional: use it as your main GitHub Pages site

If you name the repository exactly `YOUR-USERNAME.github.io`, it will be published at
`https://YOUR-USERNAME.github.io/` (no extra path) instead of a project sub-path.

## Notes

- The project flip cards flip on **hover** on desktop, and on **tap** on touch devices
  (handled by `js/script.js`).
- The site is responsive — the sidebar collapses to a top bar on narrow screens.
- No dependencies beyond two Google Fonts (Fraunces and Inter), loaded via a `<link>` tag
  in each HTML file's `<head>`.
