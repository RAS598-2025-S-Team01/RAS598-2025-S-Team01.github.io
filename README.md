# Welcome to Our Project Documentation Repository! 🌟

This repository is your one-stop hub for all the documentation needs of our project, "Fusion Robotics: A Quadruped-UR5 Robotics Testbed". Below, you'll find detailed information about each key file in our repository, ensuring you have everything at your fingertips to understand and navigate our project documentation efficiently. Let's dive in! 🚀

## Repository Structure Overview 📂

### `mkdocs.yml`

**Purpose:**
This YAML file is the heart of our MkDocs site configuration. It sets up and customizes our project documentation site, which is hosted on GitHub Pages using the sleek Material theme.

**Key Features:**

-   **Theme Customization:** Includes settings for a custom theme directory, colors, and fonts.
-   **Enhanced Navigation and Interaction:** Features like sticky tabs and editable content actions.
-   **Markdown Extensions:** Utilizes extensions like `pymdownx.arithmatex` for advanced markdown capabilities.
-   **Plugins:** Incorporates plugins like `search` and `glightbox` for improved functionality.

**Dependencies:**

-   MkDocs, Material for MkDocs, pymdownx Extensions, MathJax

**Usage Tip:** 🛠️
To tweak the site's appearance or functionality, modify the settings in this file and see the changes come to life on your GitHub Pages site.

### `requirements.txt`

**Purpose:**
Lists all necessary Python packages with specific versions to ensure our project runs smoothly and consistently across different setups.

**How to Use:**

```bash
pip install -r requirements.txt
```

This command installs all the dependencies, making sure everyone is on the same page.

### `charts.md`

**Purpose:**
Features Mermaid diagrams to visually represent the workflow involving our project's server and a quadruped robot.

**How It Helps:**
Visual aids in this file help clarify complex processes, making it easier for team members to grasp system operations.

### `index.md`

**Purpose:**
Acts as the main landing page of our documentation, outlining the project's scope, objectives, and key milestones.

**Special Remarks:**
It's a comprehensive source to understand the project's direction and weekly progress expectations.

### `robots.txt`

**Purpose:**
Instructs web robots on which parts of the site should not be accessed, crucial for controlling search engine indexing.

**Usage Example:**

```plaintext
User-agent: *
Disallow: /
```

This setup tells all robots to stay away from all sections of the site.

### `main.html`

**Purpose:**
A Django template that extends a base HTML template to customize metadata and head sections specifically for our project.

**Key Functions:**

-   Inherits and extends `base.html`.
-   Allows customization of site metadata and additional head elements.

### `pages.yml`

**Purpose:**
Defines a GitHub Actions workflow for building and deploying our MkDocs documentation to GitHub Pages upon updates to the `main` branch.

**Workflow Steps:**

-   **Checkout:** Grabs the latest code.
-   **Build:** Constructs the site using MkDocs.
-   **Deploy:** Pushes the build to GitHub Pages.

---

We hope this README helps you navigate and utilize our project documentation effectively. Should you have any questions or require further details, feel free to open an issue or submit a pull request. Let's build something amazing together! 🌐🤖
